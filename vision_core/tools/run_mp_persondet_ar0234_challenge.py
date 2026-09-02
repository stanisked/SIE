#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from vision_core.mp_persondet_challenge import run_challenge
def main() -> None:
 p=argparse.ArgumentParser(); p.add_argument('--dataset-root',type=Path,required=True); p.add_argument('--model',type=Path,required=True); p.add_argument('--reference',type=Path,required=True); p.add_argument('--report',type=Path,required=True); a=p.parse_args(); report=run_challenge(a.dataset_root,a.model,a.reference); a.report.write_text(json.dumps(report,sort_keys=True,indent=2,allow_nan=False)+'\n'); print(a.report)
if __name__ == '__main__': main()
