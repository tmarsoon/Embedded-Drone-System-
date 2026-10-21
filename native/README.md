# Hub-and-spoke drone flight simulator

A C++17 research simulator for a fleet carrying parcels from a hub to radial delivery sites and returning to the hub. It replaces the missing application sources with a runnable, tested implementation. The original project report remains unchanged.

The model couples three-dimensional aircraft dynamics, payload release and impact, time-correlated wind, sensor uncertainty, Kalman state estimation, constrained flight control, battery use, hub access, no-fly routing, and approximate acoustic exposure. Independent seeded runs produce statistical results. This is a point-mass engineering model; its parameters have not been calibrated to a particular aircraft or validated against Argonne flight data.

## Build and run

Linux needs GCC with C++17 and GNU Make. Python 3 is used for CLI tests and the experiment runner; the simulation executable has no external library dependencies.

```sh
make -j4
make test
./build/drone_sim --output results/example --trace
./build/drone_sim --runs 30 --wind 8 --gust 1.5 --output results/windy
```

The program prints JSON to stdout and exits nonzero for invalid input or execution errors. Weather rejections, missed deliveries, and timeouts are scientific outcomes reported in the JSON, rather than execution errors. Always inspect the outcome counts.

`--output` saves `summary.json` and individual results in `runs.jsonl`. `--trace` also saves `telemetry.csv` and `first_run.json` for the first seed. Use `--help` for all parameters and their units. For example:

```sh
./build/drone_sim --drones 3 --deliveries 12 --payload 0.8 \
  --wind 6 --wind-bias -2 --gps-noise 1 --gps-dropout 0.2 \
  --no-fly 50 -30 90 30 --output results/scenario --trace
```

`Embedded Drone System.sln` and the nested solution have source references restored and select C++17. Their MSVC builds require Visual Studio 2022/v143 and Windows SDK 10; Windows compilation has not been exercised in this Linux workspace.

## Research questions and measured answers

```sh
make research
# Or choose the run count and save the answers at a specific path:
python3 tools/research.py --binary build/drone_sim --runs 30 \
  --output results/research --report docs/RESEARCH_ANSWERS.md
```

The experiment runner compares weather, forecast bias, navigation degradation, payload mass, fleet size, cruise altitude, route restrictions, and launch limits. It writes measured answers, per-run data, a reproducibility manifest, and plots when Matplotlib is installed. Matplotlib is optional and is not needed to build, run, or test the C++ model.

Read the [measured questions and answers](docs/RESEARCH_ANSWERS.md), [model and assumptions](docs/MODEL.md), and [validation record](docs/VALIDATION.md). The questions are inferred from the requested research scope; they are not a statement of Argonne's official requirements.

## Algorithms and components

| Component | Implemented method | Purpose |
| --- | --- | --- |
| Aircraft and parcel motion | Fourth-order Runge–Kutta; parcel ground-event root solve | Integrate gravity, thrust, drag and landing time |
| Wind | Exact discrete Ornstein–Uhlenbeck gust update with altitude shear | Generate correlated, reproducible disturbances |
| Navigation | Six-state linear Kalman filter with Joseph covariance updates | Fuse noisy GPS and accelerometer measurements |
| Guidance | Position/velocity cascade, braking-speed constraint, integral anti-windup | Track routes with bounded speed, acceleration and thrust |
| Routes | A* with best-cost tracking and inflated no-fly columns | Find finite routes and terminate for unreachable goals |
| Payload release | Forward ballistic prediction using the forecast wind | Offset the release position for expected parcel drift |
| Fleet/hub | FIFO job queue, exclusive hub corridor, distinct cruise lanes | Coordinate shared hub access and monitor separation |
| Energy policy | Actuator-disk power estimate and conservative mission reserve | Reject infeasible launches and initiate reserve returns |
| Statistics | Independent seeded Monte Carlo, quantiles, sample SD and mean intervals | Measure uncertainty and performance tradeoffs |

All canonical application sources are in `Embedded Drone System/`. The old nested `DynamicPathPlanning.cpp` forwards to the corrected planner for compatibility; compile only the canonical translation unit in a single target.

## Verification

`make test` runs physical limiting cases, exhaustive A*/BFS comparisons, filtering and statistics checks, coupled fleet missions, no-fly and reserve policies, timestep convergence, and CLI/file tests. `make sanitize` repeats the suite with AddressSanitizer and UndefinedBehaviorSanitizer. Generated builds and experiment outputs live in ignored `build/` and `results/` directories.

The original report also mentioned RRT*, adaptive gain tuning, hardware backups, and predictive maintenance. This implementation uses the methods listed above. The reserve estimator is a transparent energy forecast, not a trained component-failure predictor; the host-side scheduler is not a hard real-time operating system.
