"""One-shot, same-cycle selected instance crop for *diagnostic* identity.

Runs only in the existing segmentation companion process, on opt-in requests.
Never promotes the selected mask into an identity decision or persistent image.
"""
from __future__ import annotations

import base64
import hashlib
import math

import cv2
import numpy as np


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def pack_selected_identity_source(
        frame, mask, bbox_xyxy, *,
        cycle_id, instance_index, rgb_arrays_sha256,
        segmentation_model_sha256):
    if (not isinstance(frame, np.ndarray) or frame.shape!=(1200,1920,3)
            or frame.dtype!=np.uint8 or not isinstance(mask,np.ndarray)
            or mask.shape!=(1200,1920) or mask.dtype!=np.bool_):
        raise ValueError("IDENTITY_SOURCE_FRAME_OR_MASK_INVALID")
    if (not isinstance(cycle_id,str) or not cycle_id
            or type(instance_index) is not int or instance_index<0
            or not isinstance(rgb_arrays_sha256,str) or len(rgb_arrays_sha256)!=64
            or not isinstance(segmentation_model_sha256,str)
            or len(segmentation_model_sha256)!=64):
        raise ValueError("IDENTITY_SOURCE_PROVENANCE_INVALID")
    if not isinstance(bbox_xyxy,(list,tuple)) or len(bbox_xyxy)!=4:
        raise ValueError("IDENTITY_SOURCE_BBOX_INVALID")
    try:
        box=[float(x) for x in bbox_xyxy]
    except (TypeError,ValueError):
        raise ValueError("IDENTITY_SOURCE_BBOX_INVALID") from None
    if not all(math.isfinite(x) for x in box):
        raise ValueError("IDENTITY_SOURCE_BBOX_INVALID")
    x1,y1,x2,y2=box
    if x1>=x2 or y1>=y2:
        raise ValueError("IDENTITY_SOURCE_BBOX_INVALID")
    x0=max(0,int(math.floor(x1)))
    y0=max(0,int(math.floor(y1)))
    x3=min(1920,int(math.ceil(x2)))
    y3=min(1200,int(math.ceil(y2)))
    if x3-x0<32 or y3-y0<64 or (x3-x0)*(y3-y0)>2_304_000:
        raise ValueError("IDENTITY_SOURCE_CROP_BOUNDS_INVALID")
    cropped=frame[y0:y3,x0:x3]
    cropped_mask=np.ascontiguousarray(mask[y0:y3,x0:x3])
    if int(np.count_nonzero(cropped_mask))<1000:
        raise ValueError("IDENTITY_SOURCE_MASK_INSUFFICIENT_PIXELS")
    success,image=cv2.imencode(".jpg",cropped,[cv2.IMWRITE_JPEG_QUALITY,92])
    if not success:
        raise ValueError("IDENTITY_SOURCE_JPEG_ENCODING_FAILED")
    img_bytes=image.tobytes()
    bit_bytes=np.packbits(cropped_mask.reshape(-1)).tobytes()
    return {
        "schema_version":"sie.shadow.identity_crop.v1",
        "scope":"MOTOR_OFF_SOURCE_CROP_NOT_IDENTITY_PROOF",
        "cycle_id":cycle_id,
        "instance_index":instance_index,
        "rgb_arrays_sha256":rgb_arrays_sha256,
        "segmentation_model_sha256":segmentation_model_sha256,
        "full_instance_mask_sha256":sha256_bytes(mask.tobytes()),
        "bbox_xyxy_px":list(bbox_xyxy),
        "crop_origin_xy_px":[x0,y0],
        "crop_shape_hw":[y3-y0,x3-x0],
        "crop_jpeg_sha256":sha256_bytes(img_bytes),
        "crop_jpeg_b64":base64.b64encode(img_bytes).decode("ascii"),
        "crop_mask_bits_sha256":sha256_bytes(bit_bytes),
        "crop_mask_bits_b64":base64.b64encode(bit_bytes).decode("ascii"),
        "lossy_rgb_crop":True,
    }


def unpack_selected_identity_source(payload):
    if (not isinstance(payload,dict)
            or payload.get("schema_version")!="sie.shadow.identity_crop.v1"
            or payload.get("scope")!="MOTOR_OFF_SOURCE_CROP_NOT_IDENTITY_PROOF"
            or not isinstance(payload.get("cycle_id"),str)
            or not isinstance(payload.get("instance_index"),int)
            or not isinstance(payload.get("rgb_arrays_sha256"),str)
            or len(payload["rgb_arrays_sha256"])!=64
            or not isinstance(payload.get("full_instance_mask_sha256"),str)
            or len(payload["full_instance_mask_sha256"])!=64):
        raise ValueError("IDENTITY_CROP_METADATA_INVALID")
    shape=payload.get("crop_shape_hw")
    if (not isinstance(shape,list) or len(shape)!=2
            or not all(type(x) is int for x in shape)
            or not 64<=shape[0]<=1200
            or not 32<=shape[1]<=1920
            or shape[0]*shape[1]>2_304_000):
        raise ValueError("IDENTITY_CROP_SHAPE_INVALID")
    try:
        jpg=base64.b64decode(payload["crop_jpeg_b64"],validate=True)
        packed=base64.b64decode(payload["crop_mask_bits_b64"],validate=True)
    except (KeyError,ValueError):
        raise ValueError("IDENTITY_CROP_ENCODING_INVALID") from None
    if (len(jpg)>4_000_000 or len(packed)!=(shape[0]*shape[1]+7)//8
            or sha256_bytes(jpg)!=payload.get("crop_jpeg_sha256")
            or sha256_bytes(packed)!=payload.get("crop_mask_bits_sha256")):
        raise ValueError("IDENTITY_CROP_HASH_OR_SIZE_INVALID")
    frame=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
    if frame is None or frame.shape!=(shape[0],shape[1],3):
        raise ValueError("IDENTITY_CROP_JPEG_INVALID")
    mask=np.unpackbits(np.frombuffer(packed,np.uint8),
                       count=shape[0]*shape[1]).reshape(shape).astype(bool)
    if int(np.count_nonzero(mask))<1000:
        raise ValueError("IDENTITY_CROP_MASK_INVALID")
    return frame,mask
