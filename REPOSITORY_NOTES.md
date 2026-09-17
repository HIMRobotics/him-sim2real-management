# Repository handoff

Canonical repository: [HIMRobotics/him-sim2real-management](https://github.com/HIMRobotics/him-sim2real-management), private.

The maintainer's existing `G1_SESSION_KIT` folder is the standalone Git repository. Its parent workspace is not a Git repository. Keep one `origin` pointing to the canonical repository and `main` tracking `origin/main`; do not initialize another repository in the parent or inside the kit. On the maintainer computer, HTTPS uses the existing GitHub CLI login through a repository-local credential helper; global Git configuration is unchanged.

Operators can use the release ZIP without Git. For a clone, the folder is `him-sim2real-management` and `bash OPEN_APP.sh` runs from its root. Normal updates are `git pull --ff-only` from the existing clone after saving/stopping a session. Local run/export folders are ignored, retained on disk, and never used as a source-code checkout. Before updating, a maintainer should inspect `git status` and preserve any local source edits.

Include the recorder, local app, documentation, tests and clearly synthetic example. Exclude real `runs/`, `exports/`, `.app/`, credentials, virtual environments, caches and robot data. The supplied `.gitignore` applies those exclusions. Exported data goes through the approved Arena handoff channel, not into the source repository.

The CI workflow tests software on Linux with Python 3.9 and 3.12. It does not connect to hardware. Keep the hardware verification status separate from a green CI result. A first measured stationary-run acceptance record should identify the SDK/firmware, robot variant, selected interface/domain and report outcome before claiming hardware validation.

No open-source license is assigned by this package. A private repository allows the owners to choose distribution terms before a public release. The Unitree SDK is referenced, not vendored. Only synthetic sample telemetry is included.
