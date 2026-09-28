# HIM Arena — Open Build Challenge (context for this build)

Source: https://arena.himrobotics.com/challenges/open-build
Prize: $2,500 · Deadline: October 15, 2026 11:59 PM ET
Winner attempt on real Unitree G1.

## What counts
Any compelling robot simulation, policy, or result video. Must be a real controller: observation in, actuator action out, simulated contact. Keyframed video alone cannot pass as policy evidence.

## Required policy contract (from challenge starters)
1. Controller in policy.py with reset() and act(observation) -> action.
2. Call act() at every control step. Bounded actuator commands must be the only code driving G1 joints after reset.
3. Do not teleport objects or change velocities after episode start — physical contact must cause the result.
4. Scripted observation-to-action is valid if labeled "scripted", never claim learned/trained or hardware readiness.
5. Causality test: this policy meets success; neutral policy on same seed fails. Log policy call count, action bounds, contact evidence, seed, both outcomes.

## Required deliverables
policy.py, scene.xml, run.py, tests/, requirements.txt, README.md, THIRD_PARTY_NOTICES.md, NEXT_IDEAS.md, run.sh
Pin dependencies. ./run.sh runs policy-contract + causality tests, then generates result.mp4 from successful episode.
Use Unitree G1 from google-deepmind/mujoco_menagerie; record exact commit; preserve license.
submission.zip + him upload to challenge open-build-challenge, robot unitree-g1, simulator mujoco.

## HIM CLI
Install: curl -fsSL https://arena.himrobotics.com/install.sh | sh
Auth: him whoami --json — if fails, pause for user login (never print tokens).
