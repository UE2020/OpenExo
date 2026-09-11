"""Summarize today's recorded controller changes without modifying recordings."""
import csv
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
saved = ROOT / "Python_GUI/Saved_Data"
tz = timezone(timedelta(hours=-4))


def epoch(clock):
    return datetime.fromisoformat("2026-09-11T" + clock).replace(tzinfo=tz).timestamp()


path = saved / "trial_20260911_151815.csv"
with path.open(encoding="utf-8-sig") as stream:
    rows = list(csv.DictReader(stream))

windows = [
    ("After recalibration", "15:18:58", "15:19:18"),
    ("PJMC before switch", "15:19:22", "15:19:42"),
    ("Zero Torque first", "15:19:45", "15:20:09"),
    ("PJMC restored first", "15:20:12", "15:20:32"),
    ("Zero Torque second", "15:21:49", "15:22:04"),
    ("PJMC restored second", "15:22:07", "15:22:27"),
]
fields = ["Desired Torque (L)", "Measured Torque (L)", "Desired Torque (R)",
          "Measured Torque (R)", "Toe FSR (L)", "Toe FSR (R)", "Exoskeleton time (seconds)"]
lines = []
for label, start, end in windows:
    subset = [r for r in rows if epoch(start) <= float(r["epoch"]) <= epoch(end)]
    lines.append(f"{label} {start} to {end}: {len(subset)} rows")
    for field in fields:
        vals = [float(r[field]) for r in subset]
        if vals:
            lines.append(f"  {field}: unique={len(set(vals))}, min={min(vals)}, max={max(vals)}")

output = "\n".join(lines) + "\n"
for name, start, end in [("trial_20260911_150523.csv", "15:05:58", "15:06:15"),
                         ("trial_20260911_150644.csv", "15:07:29", "15:07:55")]:
    with (saved / name).open(encoding="utf-8-sig") as stream:
        subset = [r for r in csv.DictReader(stream)
                  if epoch(start) <= float(r["epoch"]) <= epoch(end)]
    output += f"Unilateral Zero Torque {name} {start} to {end}: {len(subset)} rows\n"
    for field in ["Measured Torque (L)", "Measured Torque (R)"]:
        vals = [float(r[field]) for r in subset]
        if vals:
            output += f"  {field}: unique={len(set(vals))}, min={min(vals)}, max={max(vals)}\n"

(Path(__file__).parent / "zero_torque_record_results.txt").write_text(output)
print(output)
