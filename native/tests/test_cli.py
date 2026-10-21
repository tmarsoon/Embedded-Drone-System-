"""Check command-line errors, machine-readable output and telemetry consistency."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


binary = str(Path(sys.argv[1]).resolve())


def run(*args, ok=True):
    result = subprocess.run([binary, *args], capture_output=True, text=True, timeout=60)
    if ok:
        assert result.returncode == 0, result.stderr
    else:
        assert result.returncode != 0, (args, result.stdout)
        assert result.stderr.startswith("drone_sim:"), result.stderr
    return result


def valid_json(text):
    # Reject the non-standard NaN/Infinity tokens accepted by Python's default.
    return json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


assert "hub-and-spoke" in run("--help").stdout
for args in [
    ("--unknown",), ("--dt",), ("--dt", "nan"), ("--wind", "inf"),
    ("--dt", "0"), ("--drones", "0"), ("--runs", "-1"),
    ("--seed", "-1"), ("--gps-dropout", "1.1"), ("--payload", "-1"),
    ("--trace",), ("--wind", "3junk"), ("--deliveries", "1.5"),
]:
    run(*args, ok=False)

with tempfile.TemporaryDirectory(prefix="drone-cli-test-") as directory:
    first = valid_json(run("--output", directory, "--trace").stdout)
    saved = valid_json((Path(directory) / "summary.json").read_text())
    assert first == saved
    assert first["mode"] == "single"
    assert first["metrics"]["completed"] == first["metrics"]["requested"] == 6
    rows = list(csv.DictReader((Path(directory) / "telemetry.csv").open()))
    assert len(rows) > 100
    # Every trace value is finite and time is monotonic per aircraft.
    previous = {}
    for row in rows:
        identity = row["drone_id"]
        time = float(row["time_s"])
        assert math.isfinite(time) and time >= previous.get(identity, -1)
        previous[identity] = time
    individual = [valid_json(line) for line in (Path(directory) / "runs.jsonl").read_text().splitlines()]
    assert individual == [first]
    assert valid_json((Path(directory) / "first_run.json").read_text()) == first

batch = valid_json(run("--runs", "3", "--deliveries", "2").stdout)
assert batch["mode"] == "monte_carlo"
assert batch["metrics"]["runs"] == 3
assert batch["metrics"]["requested"] == batch["metrics"]["completed"] == 6
assert batch["statistics"]["delivery_error_run_mean_m"]["count"] == 3
rejected = valid_json(run("--wind", "20").stdout)
assert rejected["metrics"]["rejected"] == 6
assert rejected["metrics"]["delivered"] == 0
print("PASS CLI: invalid inputs, JSON, files, telemetry, Monte Carlo and weather rejection")
