# Old calibration branches versus the active Pi path

Reviewed on 2026-11-11 after fetching all remote branch tips. This is a
decision about code reuse, not a new calibration or an activation record.

| Branch | Old contract and artifact | Current counterpart | Decision |
| --- | --- | --- | --- |
| `feature/ar0234-intrinsic-independent-validation-v1` | V3 AR0234 independent checkerboard capture/validation, 9×6, 24.5 mm; 16 changed paths absent from Shadow branch | Active AR0234 intrinsic v5, SHA `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` | Preserve old V3 evidence on its branch. Do not copy V3 constants or revive a V3 candidate as active v5 validation. |
| `feature/ar0234-ov9281-static-extrinsic-v1` | V6 OV9281 calibration pinned in `ar0234_ov9281_static_extrinsic.py`, 9×6, 24.5 mm static target; 9 changed paths absent from Shadow branch | Pi OV9281 v7 and current AR-to-physical-left extrinsic, with versioned capture/solve/validation tools in the active branch | Preserve for forensics. Do not merge old V6 source over v7 or treat provisional static transform as activated. |
| `main` optional temperature diagnostic | One independent September commit and a test absent from Shadow branch | Active Pi control preview and Shadow candidate use separate v7 hardware diagnostics | No automatic merge into the live runner; inspect this diagnostic only if the older V5 temperature tool is needed. |

There is no currently needed independent validation tool that can be copied
unchanged into the active Pi v7/v5 chain. A new independent challenge should
use the current calibration hashes, board dimensions and frozen evidence
contract, then add a versioned tool in a separate commit. This avoids
silently mixing V3/V6 artifacts with active Pi v7. The branch ancestry and
path comparison are recorded in the
[full branch audit](https://chatgpt.com/library?file_id=libfile_76057d0d715c8191a5cfdb75777c120e&account_id=personal).
