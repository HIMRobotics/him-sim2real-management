# Validation record — September 16, 2026

Kit version: **1.2.0**, with the local browser control panel. Validation performed on the development Mac, without a connected G1.

| Check | Result |
|---|---|
| Recorder + local HTTP integration suite, Python 3.9.6 | 32 passed |
| Same suite, Python 3.12.14 | 32 passed |
| Browser: connection check → named demo run → growing counters → stop | Passed |
| Browser: outcome label → prepare verified return ZIP → download event | Passed |
| Browser screenshot review of the operator interface | Passed |
| Shell syntax, JavaScript syntax, Python compilation, fatal lint checks | Passed |
| Return ZIP → inner archive checksum → all archived payload hashes | Passed |
| Release ZIP extraction and release-file hash verification | Passed |
| Live G1 SDK / network / firmware | **Not tested here; live operator check required** |
| Actual policy motion / safe stop / camera ownership | **Not exercised; remains with the policy operators** |

The 32 automated tests cover the original recorder pipeline, guided terminal fallback, actual local HTTP request/response handling, connection gating, stale checks, duplicate recordings and duplicate app instances, immutable label corrections, attachment transfer, archive integrity, missing SDK errors, browser disconnection, app shutdown, interrupted-recorder recovery, active-run export refusal, loopback Host checks, request authentication, cross-origin rejection, path traversal rejection, and disk reserves. The optional launch wrapper remains covered by the original tests; the web app never launches a policy.

The browser checks used **synthetic demo data** and do not establish G1 compatibility or measured policy performance. File attachments were exercised through the HTTP integration tests. A live connection check and short stationary recording are still required on the real computer/robot before the session.

The app uses only Python's standard library, HTML, CSS and JavaScript. No Node runtime, web dependency installation, hosted service or account is needed to run it. Live telemetry still requires the robot's existing Unitree SDK2 Python/Cyclone DDS environment.

Version 1.2.0 adds guided SSH setup, verified recorder/policy file transfer, and a managed recorder tunnel. Additional tests execute the actual transfer and recorder-supervisor scripts locally, validate checksum failures, truncated input, unsafe archive rejection, saved connection profiles, missing-SDK gating and forbidden actions in demo/onboard mode. SSH transport is mocked in orchestration tests. The remote supervisor starts a real local HTTP server and is checked through its forwarded-host configuration, then closed by stdin EOF. No physical robot or actual remote SSH connection was tested.

Both Python versions passed all 32 tests for 1.2.0. The updated SSH panel was checked in the browser for layout, mode switching, prerequisite copy and demo-operation blocking. The full browser recording/export workflow listed above was exercised for 1.1.0; its HTTP workflow was rerun in the 1.2.0 automated suite. First live use still requires existing trusted SSH/key access, Linux Python 3.9+, working SDK imports, and a successful stationary recording/export on their robot.

Version 1.2.1 updates product branding to HIM Sim2Real Manager, removes session-specific naming, and labels Unitree G1 as the currently supported robot. Robot connectivity and recording behavior are unchanged from 1.2.0. Additional robot integrations are not implemented yet.
