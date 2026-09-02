"""Pinned OpenCV MP-PersonDet adapter for the AR0234 2D Observation MVP."""
from __future__ import annotations
import hashlib, importlib.util, math
from pathlib import Path
import cv2
import numpy as np
from .detector import ARTIFACT_SCHEMA_VERSION, CONFIDENCE_SEMANTICS, OUTPUT_CONTRACT_VERSION, DetectorArtifact
from .models import BoundingBox, PersonDetection

MODEL_SHA256="47fd5599d6fa17608f03e0eb0ae230baa6e597d7e8a2c8199fe00abea55a701f"
def artifact() -> DetectorArtifact:
 return DetectorArtifact(ARTIFACT_SCHEMA_VERSION,"mp-persondet-opencv","1","opencv_mp_persondet","2023mar",MODEL_SHA256,OUTPUT_CONTRACT_VERSION,"person",CONFIDENCE_SEMANTICS,.5,{"input_size":224,"nms_threshold":.3,"backend":"opencv_cpu","public_bbox_semantics":"FULL_BODY_ROI_FROM_MEDIAPIPE_VIRTUAL_KEYPOINTS_V1"})
def _decode_landmarks(delta, anchor, ratio, left, top):
 points=(np.asarray(delta,dtype=np.float64).reshape(4,2)/224.0+np.asarray(anchor,dtype=np.float64))*224.0
 points[:,0]=(points[:,0]-left)/ratio; points[:,1]=(points[:,1]-top)/ratio
 if not np.isfinite(points).all(): raise ValueError("non-finite virtual landmark")
 return points
def _full_body_roi(points, width, height):
 center=points[0]; radius=float(np.linalg.norm(points[0]-points[1]))
 if not math.isfinite(radius) or radius<=0: raise ValueError("invalid full-body radius")
 x1,y1,x2,y2=max(0.0,center[0]-radius),max(0.0,center[1]-radius),min(float(width),center[0]+radius),min(float(height),center[1]+radius)
 if not all(math.isfinite(x) for x in (x1,y1,x2,y2)) or x2<=x1 or y2<=y1: raise ValueError("degenerate full-body ROI")
 return BoundingBox(int(x1),int(y1),int(x2),int(y2))
def _anchors(path:Path)->np.ndarray:
 spec=importlib.util.spec_from_file_location("pinned_mp_persondet_reference",path)
 if spec is None or spec.loader is None: raise ValueError("reference unavailable")
 m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m); a=m.MPPersonDet._load_anchors(object())
 if a.shape!=(2254,2): raise ValueError("reference anchors mismatch")
 return a.astype(np.float32)
class MPPersonDetOpenCV:
 def __init__(self, model:Path, reference:Path):
  data=model.read_bytes()
  if hashlib.sha256(data).hexdigest()!=MODEL_SHA256: raise ValueError("MODEL_SHA256_MISMATCH")
  self.artifact=artifact(); self._net=cv2.dnn.readNetFromONNX(np.frombuffer(data,np.uint8)); self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV); self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU); self._anchors=_anchors(reference)
 def detect(self, frame_bgr:np.ndarray):
  h,w=frame_bgr.shape[:2]; ratio=min(224/h,224/w); rh,rw=int(h*ratio),int(w*ratio); rgb=(cv2.cvtColor(frame_bgr,cv2.COLOR_BGR2RGB).astype(np.float32)/255-.5)*2; rgb=cv2.resize(rgb,(rw,rh)); top,left=(224-rh)//2,(224-rw)//2; padded=cv2.copyMakeBorder(rgb,top,224-rh-top,left,224-rw-left,cv2.BORDER_CONSTANT,value=(0,0,0)); self._net.setInput(np.transpose(padded,(2,0,1))[None].astype(np.float32)); boxes,logits=self._net.forward(["Identity:0","Identity_1:0"])
  if boxes.shape!=(1,2254,12) or logits.shape!=(1,2254,1): raise ValueError("output contract mismatch")
  scores=1/(1+np.exp(-np.clip(logits[0,:,0].astype(np.float64),-100,100))); d=boxes[0,:,:4]/224; xy1=(d[:,:2]-d[:,2:]/2+self._anchors)*224; xy2=(d[:,:2]+d[:,2:]/2+self._anchors)*224; xy=np.c_[xy1,xy2]; xy[:,[0,2]]=(xy[:,[0,2]]-left)/ratio; xy[:,[1,3]]=(xy[:,[1,3]]-top)/ratio; xy[:,[0,2]]=np.clip(xy[:,[0,2]],0,w); xy[:,[1,3]]=np.clip(xy[:,[1,3]],0,h)
  keep=cv2.dnn.NMSBoxes(xy.tolist(),scores.tolist(),.5,.3,top_k=5000); out=[]
  for i in np.asarray(keep).reshape(-1):
   x1,y1,x2,y2=xy[int(i)]; score=float(scores[int(i)])
   if math.isfinite(score) and 0<=score<=1 and x2>x1 and y2>y1:
    roi=_full_body_roi(_decode_landmarks(boxes[0,int(i),4:],self._anchors[int(i)],ratio,left,top),w,h)
    out.append(PersonDetection(roi,score))
  return out
