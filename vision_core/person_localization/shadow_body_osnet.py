"""Motor-off OSNet x0.25 appearance embeddings from current RGB/mask evidence.

Preprocessing follows the usual OSNet/TorchReID RGB+ImageNet-normalization
convention, but this particular third-party ONNX export has no complete
published preprocessing contract. Treat all scores as exploratory until
export/preprocessing parity and negative-person separation are verified.

Model has FIXED batch size 16. Pad only during inference; never count
padding copies as samples or training/evaluation evidence.

No persistent identity mapping, no calibrated threshold, no motor I/O.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

OSNET_SHA256 = "e78604f4ccda49b8f41cd0f8f7303800ce75d2361895ebb0729513c1bf53d277"
OSNET_INPUT_SHAPE = [16, 3, 256, 128]
OSNET_OUTPUT_SHAPE = [16, 512]
RGB_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
RGB_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
SUPPORTED_CROPS = (
    "bbox_rgb",
    "mask_neutral_background",
    "mask_neutral_background_head_region_occluded",
)


def file_sha256(path: Path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1048576), b""):
            digest.update(chunk)
    return digest.hexdigest()


def body_input_tensor(frame_bgr: np.ndarray, instance_mask: np.ndarray,
                      bbox_xyxy, *, variant="bbox_rgb") -> np.ndarray:
    """Produce CHW float32, bound to the current selected instance.

    The 'head_region_occluded' variant neutralizes the upper 35% of the
    current bbox; this does NOT guarantee every face pixel is removed.
    It probes reliance on top-of-body appearance, and is not real
    back-of-person validation.
    """
    if variant not in SUPPORTED_CROPS:
        raise ValueError("UNSUPPORTED_BODY_APPEARANCE_VARIANT")
    if (not isinstance(frame_bgr, np.ndarray)
            or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3
            or frame_bgr.dtype != np.uint8
            or not isinstance(instance_mask, np.ndarray)
            or instance_mask.dtype != np.bool_
            or instance_mask.shape != frame_bgr.shape[:2]
            or not isinstance(bbox_xyxy, (list, tuple))
            or len(bbox_xyxy) != 4):
        raise ValueError("INVALID_CURRENT_RGB_BODY_MASK")
    try:
        box = [float(v) for v in bbox_xyxy]
    except (TypeError, ValueError):
        raise ValueError("INVALID_CURRENT_RGB_BODY_BBOX") from None
    if not all(math.isfinite(x) for x in box):
        raise ValueError("INVALID_CURRENT_RGB_BODY_BBOX")
    x1,y1,x2,y2=box
    if not x1 < x2 or not y1 < y2:
        raise ValueError("INVALID_CURRENT_RGB_BODY_BBOX")
    height,width=instance_mask.shape
    x0=max(0,int(math.floor(x1)))
    y0=max(0,int(math.floor(y1)))
    x3=min(width,int(math.ceil(x2)))
    y3=min(height,int(math.ceil(y2)))
    if x3-x0 < 32 or y3-y0 < 64:
        raise ValueError("BODY_CROP_TOO_SMALL")
    mask=instance_mask[y0:y3,x0:x3]
    if int(np.count_nonzero(mask)) < 1000:
        raise ValueError("BODY_MASK_INSUFFICIENT_PIXELS")
    crop=frame_bgr[y0:y3,x0:x3].copy()
    if variant != "bbox_rgb":
        crop[~mask] = (127,127,127)
    if variant == "mask_neutral_background_head_region_occluded":
        crop[:round(crop.shape[0]*.35),:] = (127,127,127)
    resized=cv2.resize(crop,(128,256),interpolation=cv2.INTER_LINEAR)
    rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)/255.0
    rgb=(rgb-RGB_MEAN)/RGB_STD
    tensor=np.ascontiguousarray(np.transpose(rgb,(2,0,1)),dtype=np.float32)
    if tensor.shape != (3,256,128) or not np.isfinite(tensor).all():
        raise ValueError("BODY_TENSOR_INVALID")
    return tensor


def normalize_embeddings(embeddings: np.ndarray) -> np.ndarray:
    arr=np.asarray(embeddings,dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != 512 or not np.isfinite(arr).all():
        raise ValueError("BODY_EMBEDDING_SHAPE_OR_FINITE_INVALID")
    norms=np.linalg.norm(arr,axis=1,keepdims=True)
    if np.any(norms <= 1.e-8):
        raise ValueError("BODY_EMBEDDING_ZERO_NORM")
    return arr/norms


class FixedBatchOsnet:
    def __init__(self,model_path: Path, *, session_factory=None):
        model_path=Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError("BODY_REID_ONNX_MODEL_UNAVAILABLE")
        self.model_sha256=file_sha256(model_path)
        if self.model_sha256 != OSNET_SHA256:
            raise ValueError("BODY_REID_MODEL_SHA256_MISMATCH")
        if session_factory is None:
            import onnxruntime as ort
            options=ort.SessionOptions()
            options.intra_op_num_threads=2
            options.inter_op_num_threads=1
            session=ort.InferenceSession(
                str(model_path),sess_options=options,
                providers=["CPUExecutionProvider"])
        else:
            session=session_factory(model_path)
        inputs=session.get_inputs()
        outputs=session.get_outputs()
        if (len(inputs)!=1 or len(outputs)!=1
                or inputs[0].shape != OSNET_INPUT_SHAPE
                or outputs[0].shape != OSNET_OUTPUT_SHAPE
                or inputs[0].type!="tensor(float)"
                or outputs[0].type!="tensor(float)"):
            raise ValueError("UNEXPECTED_FIXED_BATCH_OSNET_CONTRACT")
        self.session=session
        self.input_name=inputs[0].name
        self.output_name=outputs[0].name
        self.inference_calls=0
        self.inference_times_ms=[]

    def encode(self,inputs):
        data=np.asarray(inputs,dtype=np.float32)
        if (data.ndim!=4 or data.shape[1:]!=(3,256,128)
                or len(data)<1 or not np.isfinite(data).all()):
            raise ValueError("BODY_INPUT_BATCH_INVALID")
        out=[]
        for begin in range(0,len(data),16):
            block=data[begin:begin+16]
            valid=len(block)
            padded=np.empty((16,3,256,128),dtype=np.float32)
            padded[:valid]=block
            if valid<16:
                padded[valid:]=block[-1]
            start=perf_counter()
            values=self.session.run(
                [self.output_name],{self.input_name:padded})[0]
            duration=(perf_counter()-start)*1000.
            self.inference_calls+=1
            self.inference_times_ms.append(round(duration,2))
            values=np.asarray(values,dtype=np.float32)
            if values.shape!=(16,512):
                raise ValueError("BODY_REID_UNEXPECTED_OUTPUT_SHAPE")
            out.append(normalize_embeddings(values[:valid]))
        return np.vstack(out)


def cosine_pairs(embeddings:np.ndarray, cycles, tracks):
    if len(embeddings)!=len(cycles) or len(cycles)!=len(tracks):
        raise ValueError("BODY_PAIR_METADATA_MISMATCH")
    results=[]
    for i in range(len(cycles)):
        for j in range(i+1,len(cycles)):
            a,b=tracks[i],tracks[j]
            group=("UNTRACKED_INVOLVED" if a is None or b is None
                   else "SAME_RGB_TRACK" if a==b
                   else "DIFFERENT_RGB_TRACK")
            similarity=float(np.dot(embeddings[i],embeddings[j]))
            results.append({
                "cycles":[int(cycles[i]),int(cycles[j])],
                "tracks":[a,b],
                "group":group,
                "cosine_similarity":round(similarity,5),
            })
    return results


def similarity_summary(pairs):
    result={}
    for group in ("SAME_RGB_TRACK","DIFFERENT_RGB_TRACK","UNTRACKED_INVOLVED"):
        scores=np.asarray([item["cosine_similarity"] for item in pairs
                           if item["group"]==group],dtype=float)
        result[group]=None if len(scores)==0 else {
            "pairs":int(scores.size),
            "min":round(float(np.min(scores)),5),
            "median":round(float(np.median(scores)),5),
            "max":round(float(np.max(scores)),5)}
    return result
