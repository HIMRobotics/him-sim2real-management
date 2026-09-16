# Repository handoff

Suggested repository: `HIMRobotics/g1-session-kit`, private initially.

Include the recorder, local app, documentation, tests and clearly synthetic example. Exclude real `runs/`, `exports/`, `.app/`, credentials, virtual environments, caches and robot data. The supplied `.gitignore` applies those exclusions. Exported data goes through the approved Arena handoff channel, not into the source repository.

The CI workflow tests software on Linux with Python 3.9 and 3.12. It does not connect to hardware. Keep the hardware verification status separate from a green CI result. A first measured stationary-run acceptance record should identify the SDK/firmware, robot variant, selected interface/domain and report outcome before claiming hardware validation.

No open-source license is assigned by this package. A private repository allows the owners to choose distribution terms before a public release. The Unitree SDK is referenced, not vendored. Only synthetic sample telemetry is included.
