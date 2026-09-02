#!/usr/bin/env python3
"""Explicit, headless AR0234 2D person-localization runner."""
from __future__ import annotations
import argparse, json, sys
from datetime import datetime, timezone
from pathlib import Path
from vision_core.person_localization import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV
def args():
 p=argparse.ArgumentParser(); p.add_argument('--backend'); p.add_argument('--model',type=Path); p.add_argument('--reference',type=Path); p.add_argument('--device',type=Path,default=AR0234_BY_ID); p.add_argument('--max-frames',type=int,default=0); return p.parse_args()
def main():
 a=args()
 if a.backend != 'mp-persondet-opencv' or a.model is None or a.reference is None: print(json.dumps({'status':'BACKEND_NOT_EXPLICITLY_CONFIGURED'})); return 3
 if a.device != AR0234_BY_ID or not a.model.is_absolute() or not a.reference.is_absolute(): raise ValueError('exact stable by-id device and absolute model/reference paths are required')
 detector=MPPersonDetOpenCV(a.model,a.reference); pipeline=PersonLocalizationPipeline(detector); capture=AR0234Capture(AR0234CaptureConfig(device=AR0234_BY_ID,width=1920,height=1200,fps=30.0,fourcc='MJPG',buffer_size=1))
 capture.open()
 try:
  for _ in range(60): capture.read()
  count=0
  while not a.max_frames or count<a.max_frames:
   frame=capture.read()
   count+=1; result=pipeline.process(frame,captured_at_utc=datetime.now(timezone.utc),cycle_id=f'ar0234-live-{count}')
   print(json.dumps({'status':result.status,'timestamp':result.captured_at_utc,'bbox':None if result.bounding_box is None else result.bounding_box.to_xyxy(),'confidence':None if result.observation is None else result.observation.confidence,'detector':detector.artifact.metadata()},sort_keys=True),flush=True)
 finally: capture.close()
if __name__=='__main__':
 try: main()
 except KeyboardInterrupt: pass
 except Exception as e: print(f'ERROR: {e}',file=sys.stderr); raise SystemExit(2)
