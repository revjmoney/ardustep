# Changelog

## v2 — ready for first hardware bench testing

- Enforced distinct STEP edges with a documented 15 kstep/s ceiling.
- Preserved DDS phase across normal retargets and added reversal timing state.
- Replaced float interpolation with bounded 64-bit fixed-point calculation.
- Added coherent atomic feedback snapshots and audited ISR-shared state.
- Added startup protocol/capability handshake and fixed 1.000 ms validation.
- Latched ESTOP/watchdog faults until a disabled clear-fault transaction.
- Enforced matching sequence replies with CRC/resync/stale/timeout counters.
- Expanded bench CSV and matching latency/jitter statistics.
- Added hardware-free protocol, sequence, and step-generator model tests.

Hardware validation has not yet been performed.
