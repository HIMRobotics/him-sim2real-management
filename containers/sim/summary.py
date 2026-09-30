"""Summarize counterfactual runs from either execution backend."""

import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
root.mkdir(parents=True, exist_ok=True)
summary_path = root / "summary.json"
safety_checks = sys.argv[2] == "1"
scene = sys.argv[3] or None
rows = []
for name in sys.argv[4:]:
    path = root / name / "report.json"
    if path.is_file():
        try:
            report = json.loads(path.read_text())
        except (OSError, ValueError):
            rows.append(
                {
                    "pose": name,
                    "status": "error",
                    "reason": "invalid_report",
                    "report": str(path),
                }
            )
            continue
        rows.append(
            {
                "pose": name,
                "status": report.get("status"),
                "reason": report.get("reason"),
                "fall_check": report.get("fall_check"),
                "safety_checks": report.get("safety_checks"),
                "scene": report.get("scene"),
                "commands": report.get("commands"),
                "policy": report.get("policy"),
                "execution": report.get("execution"),
                "report": str(path),
            }
        )
    else:
        rows.append(
            {"pose": name, "status": "error", "reason": "no_report", "report": None}
        )
codes = {"within_limits": 0, "completed": 0, "violation": 1}
worst = 0
for row in rows:
    worst = max(worst, codes.get(row["status"], 2))
if scene is None:
    scenes = {row.get("scene") for row in rows if row.get("scene")}
    if len(scenes) == 1:
        scene = scenes.pop()
summary = {
    "version": 1,
    "status": {
        0: "within_limits" if safety_checks else "completed",
        1: "violation",
        2: "error",
    }[worst],
    "safety_checks": safety_checks,
    "scene": scene,
    "poses": rows,
    "hardware_authorization": False,
}
summary_path.write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
sys.exit(worst)
