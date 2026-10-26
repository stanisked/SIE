# V7 supervised bounded person approach

Experimental separate launcher; no canonical Measurement or autonomous admission.
Uses the existing static-v2 camera diagnostic, the V7 one-step observation checks
and the observed ESP32 stop trial. Nothing runs without `--execute` and a fresh
session confirmation. Default is three steps, hard cap ten. Each step requests
0.10 m and one /stop; the next step requires a new eligible frame captured after
the previous terminal observation, the same selected person box (IoU >= 0.7),
and a 0.01–0.15 m decrease in optical depth. The final step is also reobserved.
A refusal, missing frame, target change or unknown controller state ends the session
without a further move request. Separate journals are written per command.

The operator enters a freshly measured horizontal front clearance. The entry
must exceed 1.1 m plus 0.1 m per permitted step; that is an experimental request
budget, **not** a measured physical upper travel bound. Optical depth and this
clearance have different datums. No steering or path-obstacle detection exists.
The operator must keep the person stationary, the cart facing them, and the
straight path clear with power cutoff accessible. No unattended operation.
The numeric progress window is a diagnostic veto, not validated accuracy.
The 500 ms driver-declared timestamp is not independent exposure-to-actuator
certification. Network/power failures can prevent /stop; firmware stopping
distance is not independently bounded. Stop if unexpected direction occurs.

On Pi after updating the branch, first verify `NO_EXECUTION` without `--execute`.
For an operator-authorized trial, with current ESP32 LAN address and fresh measured
clearance, for example:

```bash
bash vision_core/tools/run_v7_supervised_approach.sh \
  --execute --base-url http://10.13.32.229 \
  --front-clearance-m 2.5 --max-steps 3
```

This is not a route to bypass failed V7 checks: the original one-step
`check_observation` runs on each current frame and again after controller
preflight. The original one-step launcher remains available unchanged.
