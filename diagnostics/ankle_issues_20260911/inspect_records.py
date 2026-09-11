"""Read saved trial/log evidence without modifying source records."""
import csv
from datetime import datetime, timezone
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
saved = ROOT / "Python_GUI/Saved_Data"
print("Latest trial recording coverage (GUI receipt clock):")
for path in sorted(saved.glob("trial_20260831*.csv")):
    rows = list(csv.DictReader(path.open(encoding="utf-8-sig")))
    times = [float(r["epoch"]) for r in rows]
    gaps = [b-a for a, b in zip(times, times[1:])]
    print(path.name, "rows", len(rows), "duration_s", round(times[-1]-times[0], 2),
          "median_gap_s", round(statistics.median(gaps), 4),
          "max_gap_s", round(max(gaps), 2))
    print("  columns:", ", ".join(rows[0]))

print("\nTorque recalibration requests after beginTrial, before intentional disconnect:")
for path in sorted((saved / "logs").glob("device_manager_*.log")):
    active = False
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        if "beginTrial() called" in line:
            active = True
        if "intentional disconnect" in line or "BLE transport disconnect complete" in line:
            active = False
        if active and "calibrateTorque() called" in line:
            print(path.name, line[:23])

print("\nLast recorded data for the two in-trial recalibration sessions (UTC):")
for name in ("trial_20260727_131733.csv", "trial_20260728_160813.csv"):
    rows = list(csv.DictReader((saved / name).open(encoding="utf-8-sig")))
    last = float(rows[-1]["epoch"])
    print(name, "last epoch", last, "UTC", datetime.fromtimestamp(last, timezone.utc))
