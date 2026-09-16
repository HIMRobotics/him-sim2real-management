# Validation record — September 16, 2026

Kit version: **1.1.1**, with the local browser control panel. Validation performed on the development Mac, without a connected G1.

| Check | Result |
|---|---|
| Recorder + local HTTP integration suite, Python 3.9.6 | 23 passed |
| Same suite, Python 3.12.14 | 23 passed |
| Browser: connection check → named demo run → growing counters → stop | Passed |
| Browser: outcome label → prepare verified return ZIP → download event | Passed |
| Browser screenshot review of the operator interface | Passed |
| Shell syntax, JavaScript syntax, Python compilation, fatal lint checks | Passed |
| Return ZIP → inner archive checksum → all archived payload hashes | Passed |
| Release ZIP extraction and release-file hash verification | Passed |
| Live G1 SDK / network / firmware | **Not tested here; live operator check required** |
| Actual policy motion / safe stop / camera ownership | **Not exercised; remains with the policy operators** |

The 23 automated tests cover the original recorder pipeline, guided terminal fallback, actual local HTTP request/response handling, connection gating, stale checks, duplicate recordings and duplicate app instances, immutable label corrections, attachment transfer, archive integrity, missing SDK errors, browser disconnection, app shutdown, interrupted-recorder recovery, active-run export refusal, loopback Host checks, request authentication, cross-origin rejection, path traversal rejection, and disk reserves. The optional launch wrapper remains covered by the original tests; the web app never launches a policy.

The browser checks used **synthetic demo data** and do not establish G1 compatibility or measured policy performance. File attachments were exercised through the HTTP integration tests. A live connection check and short stationary recording are still required on the real computer/robot before the session.

The app uses only Python's standard library, HTML, CSS and JavaScript. No Node runtime, web dependency installation, hosted service or account is needed to run it. Live telemetry still requires the robot's existing Unitree SDK2 Python/Cyclone DDS environment.

Version 1.1.1 rechecked Unitree's current G1 subscription contract, added the in-app/laptop/onboard connection guide, changed all handoff wording to Arena, and passed the 23-test suite on Python 3.9 again. The Python 3.12 full-suite pass was completed for 1.1.0; the 1.1.1 changes do not alter the DDS subscriber. No physical robot was connected.
