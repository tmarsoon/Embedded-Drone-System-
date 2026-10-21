#!/usr/bin/env python3
"""Run reproducible simulator experiments and write measured research answers.

The experiment runner uses only Python's standard library. Matplotlib, when
already installed, adds static figures; it is never installed by this script.
Run from the repository root after building the C++ binary.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any


PROJECT = Path(__file__).resolve().parents[1]
FIRST_SEED = 42
SCENARIOS = [
    ("baseline", [], True),
    ("calm", ["--wind", "0", "--gust", "0"], False),
    ("strong_wind", ["--wind", "8", "--gust", "1.5"], False),
    ("forecast_bias", ["--wind", "6", "--gust", "1.2", "--wind-bias", "-2"], False),
    ("gps_degraded", ["--gps-noise", "1", "--gps-dropout", "0.2"], True),
    ("heavy_payload", ["--payload", "0.8"], False),
    ("single_drone", ["--drones", "1"], False),
    ("six_drones", ["--drones", "6"], False),
    ("busy_hub", ["--deliveries", "30", "--duration", "1800"], False),
    ("lower_altitude", ["--altitude", "20"], False),
    ("no_fly", ["--no-fly", "50", "-30", "90", "30"], True),
    ("battery_limit", ["--battery", "1"], False),
    ("weather_limit", ["--wind", "20"], False),
]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_manifest() -> dict[str, str]:
    paths = [p for p in (PROJECT / "Embedded Drone System").rglob("*")
             if p.is_file() and p.suffix in {".cpp", ".h", ".hpp"}]
    paths += [p for p in (PROJECT / "Makefile", PROJECT / "CMakeLists.txt", Path(__file__))
              if p.is_file()]
    return {str(p.relative_to(PROJECT)): sha256(p) for p in sorted(paths)}


def combined_hash(files: dict[str, str]) -> str:
    data = "".join(f"{name}\0{digest}\n" for name, digest in sorted(files.items()))
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def load_json(text: str) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise ValueError(f"invalid nonfinite JSON token: {value}")

    value = json.loads(text, parse_constant=reject_nonfinite)
    if not isinstance(value, dict):
        raise ValueError("simulator JSON must be an object")
    return value


def dump_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def quantile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = probability * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (index - lower) * (ordered[upper] - ordered[lower])


def normal_ci(values: list[float]) -> tuple[float | None, float | None]:
    if len(values) < 2:
        return None, None
    center = statistics.fmean(values)
    half_width = 1.959963984540054 * statistics.stdev(values) / math.sqrt(len(values))
    return center - half_width, center + half_width


def fmt(value: float | None, places: int = 2) -> str:
    return "n/a" if value is None else f"{value:,.{places}f}"


def percent(numerator: int, denominator: int) -> str:
    return "n/a" if not denominator else f"{100.0 * numerator / denominator:.1f}%"


def finite_values(values: list[Any]) -> list[float]:
    return [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]


def collect_metrics(runs: list[dict[str, Any]]) -> dict[str, Any]:
    errors: list[float] = []
    run_means: list[float] = []
    throughput: list[float] = []
    energy: list[float] = []
    duration: list[float] = []
    success_rates: list[float] = []
    counts = {key: 0 for key in ("requested", "delivered", "completed", "within_radius",
                                "rejected", "emergency_returns", "separation_events")}
    for run in runs:
        metrics = run["metrics"]
        for key in counts:
            counts[key] += metrics[key]
        impact_errors = [float(m["impact_error_m"]) for m in run["missions"] if m["delivered"]]
        errors.extend(impact_errors)
        if impact_errors:
            run_means.append(statistics.fmean(impact_errors))
        seconds = float(metrics["duration_s"])
        throughput.append(metrics["completed"] * 3600.0 / seconds if seconds > 0.0 else 0.0)
        energy.append(float(metrics["energy_wh"]))
        duration.append(seconds)
        if metrics["requested"]:
            success_rates.append(metrics["within_radius"] / metrics["requested"])
    minimum_distances = finite_values([r["metrics"]["minimum_separation_m"] for r in runs])
    maximum_noise = finite_values([r["metrics"]["max_noise_db"] for r in runs])
    equivalent_noise = finite_values([r["metrics"]["noise_laeq_db"] for r in runs])
    counts.update({
        "runs": len(runs),
        "timed_out_runs": sum(bool(r["metrics"]["timed_out"]) for r in runs),
        "error_count": len(errors),
        "error_mean": mean(errors), "error_p50": quantile(errors, 0.5),
        "error_p95": quantile(errors, 0.95),
        "run_error_mean": mean(run_means), "run_error_ci95": normal_ci(run_means),
        "run_error_count": len(run_means),
        "success_mean": mean(success_rates), "success_ci95": normal_ci(success_rates),
        "energy_mean_wh": mean(energy), "energy_p95_wh": quantile(energy, 0.95),
        "duration_mean_s": mean(duration), "duration_p95_s": quantile(duration, 0.95),
        "throughput_mean_per_hour": mean(throughput),
        "minimum_separation_m": min(minimum_distances) if minimum_distances else None,
        "max_noise_db": max(maximum_noise) if maximum_noise else None,
        # Arithmetic mean of run-level dB values is descriptive across trials;
        # acoustic sources within each run were summed as linear intensities.
        "noise_laeq_mean_db": mean(equivalent_noise),
    })
    return counts


def execute_case(binary: Path, output: Path, name: str, options: list[str], runs: int,
                 trace: bool, timeout: float, manifest: dict[str, Any]) -> dict[str, Any]:
    directory = output / name
    command = [str(binary), "--seed", str(FIRST_SEED), "--runs", str(runs),
               *options, "--output", str(directory)]
    if trace:
        command.append("--trace")
    entry: dict[str, Any] = {"name": name, "command": command, "runs": runs,
                             "seed_first": FIRST_SEED, "seed_last": FIRST_SEED + runs - 1}
    manifest["experiments"].append(entry)
    process = subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)
    entry["exit_code"] = process.returncode
    if process.returncode:
        entry["stderr"] = process.stderr[-4000:]
        raise RuntimeError(f"experiment {name} failed with exit {process.returncode}: {process.stderr[-2000:]}")
    summary = load_json(process.stdout)
    saved_summary = load_json((directory / "summary.json").read_text(encoding="utf-8"))
    if summary != saved_summary:
        raise RuntimeError(f"experiment {name}: stdout and saved summary disagree")
    individual = [load_json(line) for line in (directory / "runs.jsonl").read_text(encoding="utf-8").splitlines()
                  if line.strip()]
    if len(individual) != runs:
        raise RuntimeError(f"experiment {name}: expected {runs} individual results, got {len(individual)}")
    for index, run in enumerate(individual):
        if run["config"]["seed"] != FIRST_SEED + index:
            raise RuntimeError(f"experiment {name}: wrong seed at run {index}")
        metrics = run["metrics"]
        if len(run["missions"]) != metrics["requested"]:
            raise RuntimeError(f"experiment {name}: mission count does not match requests")
        for field in ("delivered", "completed", "within_radius"):
            mission_field = field
            if sum(bool(m[mission_field]) for m in run["missions"]) != metrics[field]:
                raise RuntimeError(f"experiment {name}: inconsistent {field} accounting")
        if not 0 <= metrics["within_radius"] <= metrics["delivered"] <= metrics["requested"]:
            raise RuntimeError(f"experiment {name}: invalid delivery accounting")
    entry["config"] = summary["config"]
    entry["summary_sha256"] = sha256(directory / "summary.json")
    entry["runs_sha256"] = sha256(directory / "runs.jsonl")
    if trace:
        entry["trace_sha256"] = sha256(directory / "telemetry.csv")
    return {"name": name, "directory": directory, "summary": summary,
            "runs": individual, "metrics": collect_metrics(individual)}


def read_trace(case: dict[str, Any]) -> list[dict[str, str]]:
    path = case["directory"] / "telemetry.csv"
    with path.open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def navigation_rmse(case: dict[str, Any]) -> dict[str, Any]:
    rows = read_trace(case)
    first_active: dict[int, float] = {}
    squared_errors: list[float] = []
    inactive = {"parked", "idle", "complete", "completed", "rejected", "done"}
    for row in rows:
        if row["phase"].lower() in inactive:
            continue
        drone_id, time = int(row["drone_id"]), float(row["time_s"])
        first_active.setdefault(drone_id, time)
        if time < first_active[drone_id] + 2.0:
            continue
        squared_errors.append(sum((float(row[f"{axis}_m"]) - float(row[f"estimated_{axis}_m"])) ** 2
                                  for axis in "xyz"))
    return {"samples": len(squared_errors),
            "position_3d_rmse_m": math.sqrt(statistics.fmean(squared_errors)) if squared_errors else None,
            "raw_gps_theoretical_3d_rmse_m": math.sqrt(3.0) * case["summary"]["config"]["gps_sigma_m"],
            "seed": FIRST_SEED}


def no_fly_observations(case: dict[str, Any]) -> dict[str, int]:
    zones = case["summary"]["config"]["no_fly_zones_m"]
    tested = 0
    inside = 0
    for row in read_trace(case):
        if float(row["z_m"]) <= 0.5:
            continue
        tested += 1
        x, y = float(row["x_m"]), float(row["y_m"])
        if any(xmin <= x <= xmax and ymin <= y <= ymax for xmin, ymin, xmax, ymax in zones):
            inside += 1
    return {"airborne_samples": tested, "inside_zone_samples": inside, "seed": FIRST_SEED}


def optional_figures(output: Path, cases: dict[str, dict[str, Any]]) -> tuple[list[Path], str]:
    os.environ.setdefault("MPLCONFIGDIR", str(output / ".matplotlib-cache"))
    os.environ.setdefault("XDG_CACHE_HOME", str(output / ".font-cache"))
    Path(os.environ["XDG_CACHE_HOME"]).mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return [], "Matplotlib was unavailable; no plots were generated. JSON/JSONL/CSV and this report are complete."

    rows = read_trace(cases["baseline"])
    figure, (xy, altitude) = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for drone_id in sorted({int(row["drone_id"]) for row in rows}):
        selected = [row for row in rows if int(row["drone_id"]) == drone_id]
        xy.plot([float(r["x_m"]) for r in selected], [float(r["y_m"]) for r in selected],
                label=f"Drone {drone_id}", linewidth=1)
        altitude.plot([float(r["time_s"]) for r in selected], [float(r["z_m"]) for r in selected],
                      label=f"Drone {drone_id}", linewidth=1)
    first = cases["baseline"]["runs"][0]
    destinations = [mission["destination_m"] for mission in first["missions"]]
    impacts = [mission["impact_m"] for mission in first["missions"] if mission["delivered"]]
    if destinations:
        xy.scatter([p[0] for p in destinations], [p[1] for p in destinations], marker="x", c="black", label="Spoke")
    if impacts:
        xy.scatter([p[0] for p in impacts], [p[1] for p in impacts], s=18, facecolors="none", edgecolors="red", label="Impact")
    xy.scatter([0], [0], marker="s", c="black", label="Hub")
    xy.set(xlabel="East (m)", ylabel="North (m)", title="Seed 42: aircraft paths and parcel impacts")
    xy.set_aspect("equal", adjustable="datalim")
    xy.legend(fontsize=8)
    altitude.set(xlabel="Time (s)", ylabel="Altitude (m)", title="Altitude lanes and shared hub access")
    altitude.legend(fontsize=8)
    for axis in (xy, altitude):
        axis.grid(alpha=0.2)
    flight_path = output / "fleet_paths.png"
    figure.savefig(flight_path, dpi=160)
    plt.close(figure)

    names = ["baseline", "calm", "strong_wind", "gps_degraded", "heavy_payload", "single_drone", "six_drones", "lower_altitude"]
    comparisons = [("error_p95", "Parcel impact error p95 (m)"),
                   ("energy_mean_wh", "Mean fleet energy (Wh)"),
                   ("throughput_mean_per_hour", "Mean completed missions / hour"),
                   ("noise_laeq_mean_db", "Mean run-level modeled LAeq (dB)")]
    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    for axis, (key, label) in zip(axes.flat, comparisons):
        values = [cases[name]["metrics"][key] for name in names]
        axis.bar(range(len(names)), [value if value is not None else math.nan for value in values])
        axis.set_xticks(range(len(names)), [name.replace("_", "\n") for name in names], fontsize=8)
        axis.set_ylabel(label)
        axis.grid(axis="y", alpha=0.2)
    compare_path = output / "scenario_comparison.png"
    figure.savefig(compare_path, dpi=160)
    plt.close(figure)
    return [flight_path, compare_path], "Figures use the actual C++ outputs. Acoustic values are model estimates, not measured field sound."


def link(path: Path, report: Path) -> str:
    return Path(os.path.relpath(path, report.parent)).as_posix()


def write_report(report: Path, output: Path, cases: dict[str, dict[str, Any]],
                 convergence: dict[str, dict[str, Any]], navigation: dict[str, Any],
                 no_fly: dict[str, int], figures: list[Path], figure_note: str,
                 manifest: dict[str, Any]) -> None:
    b = cases["baseline"]["metrics"]
    c = cases["baseline"]["summary"]["config"]
    count = manifest["runs_per_scenario"]
    lines = [
        "# Hub-and-spoke drone simulation: measured research answers", "",
        "These are candidate research questions inferred from the requested hub-and-spoke drone simulation. "
        "They are not official Argonne requirements or statements from Argonne. Answers below were computed "
        "from the runnable C++ model; they do not establish field performance.", "",
        f"Each main scenario uses {count} seeded repetitions, seeds {FIRST_SEED}–{FIRST_SEED + count - 1}. "
        f"The baseline has {c['drones']} aircraft, {c['deliveries']} parcel requests, "
        f"{fmt(c['spoke_radius_m'], 0)} m spokes, cruise lanes beginning at {fmt(c['cruise_altitude_m'], 0)} m "
        f"with {fmt(c['altitude_spacing_m'], 0)} m spacing, {fmt(c['wind_speed_mps'])} m/s surface wind, "
        f"and {fmt(c['payload_mass_kg'])} kg parcels. The step is {fmt(c['dt_s'], 3)} s.", "",
        "## Measured scenario results", "",
        "Counts are totals across repetitions. Completed means parcel impact and aircraft return; delivered means "
        "a parcel reached the ground, which can still miss the acceptance radius. In-radius is measured against "
        "all requested jobs, including rejected or failed jobs. Energy is the mean total fleet energy per repetition. "
        "Impact percentiles are conditional on parcels that impacted; omitted impacts are not assigned zero error.", "",
        "| Scenario | Requested / delivered / completed / in-radius | In-radius % | Energy mean Wh | Impact p50 / p95 m | Completed/h | Mean LAeq dB | Min separation m | Events / timeouts |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, _, _ in SCENARIOS:
        m = cases[name]["metrics"]
        lines.append(f"| {name} | {m['requested']} / {m['delivered']} / {m['completed']} / {m['within_radius']} | "
                     f"{percent(m['within_radius'], m['requested'])} | {fmt(m['energy_mean_wh'])} | "
                     f"{fmt(m['error_p50'])} / {fmt(m['error_p95'])} | {fmt(m['throughput_mean_per_hour'])} | "
                     f"{fmt(m['noise_laeq_mean_db'])} | {fmt(m['minimum_separation_m'])} | "
                     f"{m['separation_events']} / {m['timed_out_runs']} |")
    lines += ["", "Exact configurations, raw outcomes, and commands accompany the report; scenario names are labels, "
              "not claims of general operating limits.", "", "## Questions and answers", ""]

    single = cases["single_drone"]["metrics"]
    six = cases["six_drones"]["metrics"]
    busy = cases["busy_hub"]["metrics"]
    lines += [
        "### 1. What hub throughput does the fleet achieve, and does adding aircraft help?", "",
        f"The baseline completed {b['completed']}/{b['requested']} requested missions and averaged "
        f"{fmt(b['throughput_mean_per_hour'])} completed missions/hour. One aircraft averaged "
        f"{fmt(single['throughput_mean_per_hour'])}/hour; six averaged {fmt(six['throughput_mean_per_hour'])}/hour. "
        f"With 30 queued requests per run, the busy-hub case completed {busy['completed']}/{busy['requested']} "
        f"and averaged {fmt(busy['throughput_mean_per_hour'])}/hour. "
        "The model serializes takeoff and landing corridor ownership and gives each aircraft an altitude lane. "
        "This is finite-workload throughput, not a measured steady-state arrival capacity or an optimized fleet size.", "",
        "### 2. How accurately does a parcel reach a spoke?", "",
        f"Baseline parcels had conditional median error {fmt(b['error_p50'])} m, p95 {fmt(b['error_p95'])} m, "
        f"and pooled mean {fmt(b['error_mean'])} m over {b['error_count']} impacts. "
        f"{b['within_radius']}/{b['requested']} requests ({percent(b['within_radius'], b['requested'])}) "
        f"landed within the configured {fmt(c['delivery_radius_m'])} m radius. "
        "Release offsets are computed with the same ballistic drag equations using forecast wind; the actual parcel "
        "experiences the evolving wind field and inherits aircraft velocity. Parcel delivery and safe aircraft return "
        "are reported separately.", "",
        "### 3. What changes when wind or wind forecasts are worse?", "",
    ]
    for name in ("calm", "strong_wind", "forecast_bias"):
        m = cases[name]["metrics"]
        lines += [f"- `{name}`: p95 impact error {fmt(m['error_p95'])} m; in-radius "
                  f"{m['within_radius']}/{m['requested']}; mean energy {fmt(m['energy_mean_wh'])} Wh; "
                  f"rejected launches {m['rejected']}; emergency returns {m['emergency_returns']}; "
                  f"timeouts {m['timed_out_runs']}."]
    lines += ["", "The wind model combines a directional mean, altitude shear, and seeded Ornstein–Uhlenbeck gusts. "
              "The forecast omits gusts; the forecast-bias scenario additionally underestimates mean speed by 2 m/s. "
              "These scenarios isolate model sensitivities and do not reproduce a measured site weather field.", "",
              "### 4. Does Kalman filtering help when GPS is noisy or intermittent?", ""]
    for name in ("baseline", "gps_degraded"):
        n = navigation[name]
        lines += [f"- `{name}`, first seed {FIRST_SEED}: fused position 3D RMSE "
                  f"{fmt(n['position_3d_rmse_m'], 3)} m over {n['samples']} airborne samples after the first "
                  f"2 s per aircraft; configured raw GPS theoretical 3D RMSE "
                  f"{fmt(n['raw_gps_theoretical_3d_rmse_m'], 3)} m."]
    lines += ["", "The reference GPS value is √3 times the configured per-axis Gaussian standard deviation, "
              "not a separately measured raw-GPS trace. The filter predicts position/velocity from accelerometer "
              "measurements and corrects with noisy GPS when available; it does not consume truth velocity. "
              "RMSE against simulation truth is a closed-model consistency check, not field validation.", "",
              "### 5. How do payload mass and release dynamics affect the aircraft and parcel?", ""]
    heavy = cases["heavy_payload"]["metrics"]
    lines += [f"Increasing parcel mass from {fmt(c['payload_mass_kg'])} to 0.80 kg changed mean fleet energy from "
              f"{fmt(b['energy_mean_wh'])} to {fmt(heavy['energy_mean_wh'])} Wh and p95 impact error from "
              f"{fmt(b['error_p95'])} to {fmt(heavy['error_p95'])} m. The heavy case completed "
              f"{heavy['completed']}/{heavy['requested']} requests, with {heavy['emergency_returns']} emergency returns. "
              "Release reduces aircraft mass immediately, and the controller changes its mass-dependent gravity "
              "compensation. Parcel drag area is held fixed in this sensitivity study; real packages of different "
              "mass can also change geometry, center of gravity, inertia, and separation aerodynamics.", "",
              "### 6. Do routing restrictions change service or cause no-fly intrusion?", ""]
    nf = cases["no_fly"]["metrics"]
    lines += [f"With the no-fly column x∈[50,90] m, y∈[−30,30] m, the fleet completed "
              f"{nf['completed']}/{nf['requested']} requests, averaged {fmt(nf['energy_mean_wh'])} Wh, and "
              f"{fmt(nf['throughput_mean_per_hour'])} completed missions/hour. In the recorded seed-{FIRST_SEED} "
              f"trace, {no_fly['inside_zone_samples']}/{no_fly['airborne_samples']} airborne sample points were inside "
              "the column. This sampled-position check does not prove continuous swept-volume clearance or test "
              "every Monte Carlo path. The planner uses bounded A* exploration with obstacle clearance; moving "
              "people, vehicle shapes, and uncertain obstacle maps need additional modeling.", "",
              "### 7. Does hub coordination maintain aircraft separation?", "",
              f"Baseline minimum recorded airborne separation was {fmt(b['minimum_separation_m'])} m, with "
              f"{b['separation_events']} threshold events. The six-aircraft case reached "
              f"{fmt(six['minimum_separation_m'])} m and recorded {six['separation_events']} events. "
              f"The configured threshold is {fmt(c['min_separation_m'])} m. These are actual simulated state "
              f"measurements. The 30-job busy-hub case recorded {busy['separation_events']} events, with minimum "
              f"separation {fmt(busy['minimum_separation_m'])} m and {busy['timed_out_runs']} timed-out runs. "
              "Protected ascent/descent-column reservations check remaining routes before granting hub access, "
              "and waiting arrivals have priority over new launches. These results are not guaranteed separation "
              "from the routing policy. Altitude lanes and exclusive hub "
              "corridor access do not replace full continuous collision prediction, wake interaction, or airspace "
              "traffic management.", "",
              "### 8. What happens when a flight should not launch?", ""]
    for name in ("battery_limit", "weather_limit"):
        m = cases[name]["metrics"]
        lines += [f"- `{name}`: {m['rejected']}/{m['requested']} rejected, {m['completed']} completed, "
                  f"{m['emergency_returns']} emergency returns, {m['timed_out_runs']} timed-out runs."]
    lines += ["", "The energy policy estimates total planned route travel, climb/descent, and service hover with "
              "a 1.3 uncertainty factor and a reserve against rated battery capacity. In-flight reserve monitoring "
              "can abort a delivery and return. This is an energy forecast, not a trained predictive maintenance "
              "model; cell aging, voltage sag, temperature-dependent capacity, motor faults, and forecast-tail risk "
              "are not inferred from real maintenance records.", "",
              "### 9. How does flight altitude affect modeled neighborhood noise?", ""]
    lower = cases["lower_altitude"]["metrics"]
    lines += [f"At observer {c['noise_observer_m']} m, baseline mean run-level LAeq was "
              f"{fmt(b['noise_laeq_mean_db'])} dB and the largest run maximum was {fmt(b['max_noise_db'])} dB. "
              f"Beginning lanes at 20 m produced {fmt(lower['noise_laeq_mean_db'])} dB mean LAeq and "
              f"{fmt(lower['max_noise_db'])} dB maximum. Simultaneous aircraft are summed as acoustic intensities "
              "inside each run; the scenario table averages run-level dB values across repetitions. The source "
              "level, power scaling, spherical spreading, and absorption are simplified assumptions. No measured "
              "frequency spectrum, ground reflection, shielding, rotor directivity, or site calibration is present. "
              "These values must not be treated as measured environmental compliance results.", "",
              "### 10. How confident are the reported outcomes?", ""]
    low, high = b["run_error_ci95"]
    ci_text = (f"[{fmt(low, 3)}, {fmt(high, 3)}] m" if low is not None else "unavailable with fewer than two independent runs")
    lines += [f"The baseline mean of independent run-mean impact errors was {fmt(b['run_error_mean'], 3)} m "
              f"over {b['run_error_count']} runs with impacts. Its approximate 95% normal mean interval is {ci_text}. "
              "Parcels in one run share weather, so pooled parcel percentiles are descriptive and their naive "
              "parcel-level confidence interval is not used for the research answer. The mean interval describes "
              "Monte Carlo sampling uncertainty under this model; it excludes uncertain model parameters and "
              "does not predict the next parcel. Small-run normal intervals are weak, and zero observed failures "
              "does not establish zero risk. Error summaries are conditional on impact and must be read beside "
              "the rejection/failure counts.", "",
              "### 11. Are timestep and numerical artifacts small enough to trust these comparisons?", ""]
    coarse = convergence["convergence_dt_0p1"]["metrics"]
    fine = convergence["convergence_dt_0p05"]["metrics"]
    lines += ["The timestep check is a separate one-aircraft, one-parcel, calm run with GPS/accelerometer noise "
              "and GPS dropouts disabled. Stochastic traces at different steps are deliberately not treated as paired weather.", "",
              "| Step s | Completed | Duration s | Energy Wh | Impact error m |",
              "|---:|---:|---:|---:|---:|",
              f"| 0.10 | {coarse['completed']} | {fmt(coarse['duration_mean_s'], 4)} | {fmt(coarse['energy_mean_wh'], 6)} | {fmt(coarse['error_mean'], 6)} |",
              f"| 0.05 | {fine['completed']} | {fmt(fine['duration_mean_s'], 4)} | {fmt(fine['energy_mean_wh'], 6)} | {fmt(fine['error_mean'], 6)} |", "",
              "These two points quantify local timestep sensitivity; they do not establish continuum convergence "
              "or validate the physical assumptions. Unit tests additionally compare vacuum ballistic motion and "
              "hover balance with analytic expectations. Finite-step scheduling, release, and landing thresholds "
              "can shift event times even when the force integrator is accurate.", "",
              "### 12. What is still needed for a credible Argonne study or real-flight comparison?", "",
              "The working model supplies a reproducible experiment harness, gravity/drag flight and parcel "
              "dynamics, stochastic wind, Kalman state estimation, mass-aware control, routing, dispatch, energy "
              "policy, acoustics, and measured uncertainty summaries. A defensible site study additionally needs "
              "actual research requirements, surveyed hub/pad/spoke geometry, terrain and no-fly maps, calibrated "
              "aircraft thrust/drag/power and payload coefficients, measured winds and sensor errors, measured "
              "acoustic spectra, realistic demand/service times, and held-out flight logs. The vehicle is a point "
              "mass with commanded thrust, not a six-degree-of-freedom rigid body with motor/attitude transients, "
              "rotor wakes, release mechanisms, and ground effect. Model calibration and independent validation "
              "must precede claims of real-world accuracy; no output here is an official Argonne acceptance criterion.", "",
              "## Reproducibility and artifacts", "",
              f"- Binary SHA256: `{manifest['binary_sha256']}`.",
              f"- Source bundle SHA256: `{manifest['source_bundle_sha256']}` (individual files in the manifest).",
              f"- [Manifest and exact command arrays]({link(output / 'manifest.json', report)}).",
              f"- [Machine-readable comparison metrics]({link(output / 'comparison.json', report)}).",
              f"- [Baseline raw runs]({link(output / 'baseline' / 'runs.jsonl', report)}).",
              f"- [Baseline telemetry CSV]({link(output / 'baseline' / 'telemetry.csv', report)}).",
              f"- [Degraded-GPS telemetry CSV]({link(output / 'gps_degraded' / 'telemetry.csv', report)}).", "",
              "Reproduce from the repository root:", "", "```sh",
              f"python3 tools/research.py --binary build/drone_sim --output results/research --runs {count}",
              "```", "", figure_note, ""]
    for figure in figures:
        lines += [f"![{figure.stem.replace('_', ' ')}]({link(figure, report)})", ""]
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("build/drone_sim"))
    parser.add_argument("--output", type=Path, default=Path("results/research"))
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--timeout", type=float, default=180.0, help="wall-time limit per scenario in seconds")
    args = parser.parse_args()
    if not 1 <= args.runs <= 1000:
        parser.error("--runs must be in [1,1000]")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    binary = args.binary.resolve()
    output = args.output.resolve()
    report = args.report.resolve() if args.report else output / "REPORT.md"
    if not binary.is_file():
        parser.error(f"binary not found: {binary}; build it first")
    output.mkdir(parents=True, exist_ok=True)
    sources = source_manifest()
    manifest: dict[str, Any] = {
        "schema_version": 1, "status": "running", "started_utc": utc_now(),
        "binary": str(binary), "binary_sha256": sha256(binary),
        "source_files_sha256": sources, "source_bundle_sha256": combined_hash(sources),
        "runs_per_scenario": args.runs, "independent_seeds": list(range(FIRST_SEED, FIRST_SEED + args.runs)),
        "experiments": [],
    }
    dump_json(output / "manifest.json", manifest)
    try:
        cases: dict[str, dict[str, Any]] = {}
        for name, options, trace in SCENARIOS:
            print(f"Running {name} ({args.runs} seeds)", file=sys.stderr, flush=True)
            cases[name] = execute_case(binary, output, name, options, args.runs, trace, args.timeout, manifest)
            dump_json(output / "manifest.json", manifest)
        convergence = {}
        quiet = ["--drones", "1", "--deliveries", "1", "--wind", "0", "--gust", "0",
                 "--gps-noise", "0", "--accel-noise", "0", "--gps-dropout", "0"]
        for name, step in (("convergence_dt_0p1", "0.1"), ("convergence_dt_0p05", "0.05")):
            print(f"Running {name} (noiseless numerical check)", file=sys.stderr, flush=True)
            convergence[name] = execute_case(binary, output, name, [*quiet, "--dt", step], 1,
                                             False, args.timeout, manifest)
        navigation = {name: navigation_rmse(cases[name]) for name in ("baseline", "gps_degraded")}
        no_fly = no_fly_observations(cases["no_fly"])
        comparison = {"scenarios": {name: case["metrics"] for name, case in cases.items()},
                      "convergence": {name: case["metrics"] for name, case in convergence.items()},
                      "navigation_first_seed": navigation, "no_fly_first_seed": no_fly}
        dump_json(output / "comparison.json", comparison)
        figures, figure_note = optional_figures(output, cases)
        if sha256(binary) != manifest["binary_sha256"] or source_manifest() != sources:
            raise RuntimeError("binary or source changed during experiments; rebuild and rerun for consistent provenance")
        manifest.update({"status": "complete", "finished_utc": utc_now(), "report": str(report),
                         "figure_paths": [str(p) for p in figures]})
        dump_json(output / "manifest.json", manifest)
        write_report(report, output, cases, convergence, navigation, no_fly, figures, figure_note, manifest)
        print(f"Wrote {report}")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
        manifest.update({"status": "failed", "finished_utc": utc_now(), "error": str(error)})
        dump_json(output / "manifest.json", manifest)
        print(f"research.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
