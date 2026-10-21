# Integration and validation record

## Architecture

`application.Launcher` starts the JavaFX tabs. `FlightDashboard` creates an immutable `SimulationRequest` and runs `NativeSimulator` in a background task. The bridge starts C++ with a list of arguments, records each invocation in a new directory, and waits with a wall-time limit. Cancellation terminates the child and descendants; partial runs retain their files.

C++ is authoritative for trajectories, estimator states, impacts, dispatch, energy, separation, and sound. Java validates schema version 1, requested configuration, mission accounting, nullable impacts, finite telemetry and monotonic per-aircraft times. Telemetry uses metres, seconds, metres/second, newtons and watt-hours in local east/north/up coordinates. No network service or shell interpolation is involved.

The four template hubs supply geographic context. A bounded local projection supplies geographic coordinates within 10 km of a hub. The globe provides selection/context; the trajectory view renders native telemetry. No intercontinental transport, site-specific terrain/weather, or spherical-Earth dynamics are implied.

## Changes from the uploaded template

The entry point now opens the integrated dashboard. It no longer starts random orbit motion, synchronous deployment loops, or roster-wide repeated logging. Those demonstration classes remain at their original paths for reference. The new six-state C++ Kalman estimator provides flight navigation; the retained Java scalar filter is not the flight estimator.

CSV loading handles quoted commas, escaped quotes, embedded newlines, UTF-8 BOMs, and actual header names. Source order is preserved unless a seeded shuffle is explicitly requested. All 3,473 supplied historical records load: 2,426 Army, 734 Navy, 295 Marines, 17 Air Force and one Coast Guard record. The roster searches the supplied public data without treating it as live operational information.

Legacy logging appends under `runs/logs/` and does not open historical uploaded logs. Repeated base-node generation and visualization setup no longer accumulate duplicates. Maven replaces absolute machine-specific library paths. The original uploaded archive is preserved outside the checkout; a source/resource hash inventory is included.

## Executed verification

Linux toolchain: GCC 14.2, C++17, Python 3.12, JDK 21, Maven 3.9.9, JavaFX 21.0.13, CMake 3.31.10. JavaFX used Xvfb and Mesa llvmpipe OpenGL with 3D support.

| Check | Observed result |
| --- | --- |
| GNU Make native build/tests | 28/28 test groups plus CLI/file checks passed |
| Planner oracle | 41,472 A*/independent-BFS comparisons passed |
| CMake build/CTest | Native suite and CLI tests passed (2/2 CTest entries) |
| Maven verify | 18 tests passed, zero failures/errors/skips; packaged successfully |
| Actual native bridge | Six missions returned/accepted; telemetry, missions, geographic report loaded, including paths with spaces |
| Boundary/error handling | Invalid input/schema/accounting/CSV/nonfinite or duplicate telemetry rejected; existing output preserved |
| Child lifecycle | Actual native timeout and cancellation passed |
| Uploaded data | All 3,473 records and branch counts verified; quoted/multiline CSV and append-only logging checked |
| Desktop smoke | Actual Run pipeline: six returned/accepted, zero separation events, 5,440 telemetry rows; playback and charts rendered; PNG saved |
| Prior native validation | Unchanged engine passed ASan/UBSan and 392 research executions before directory relocation |

`docs/images/flight-lab.png` shows the actual desktop run. Research manifests/hashes refer to the pre-integration root layout, preserved as reference artifacts. Reproduce current paths using the top-level README. Windows scripts/CI are included; Windows execution was not observed in this Linux workspace. Check GitHub for CI results after publication.

## Reproduce

From the root on Linux:

```sh
make test
mvn -B -C verify
bash scripts/smoke-ui.sh
```

The opt-in smoke mode runs through the normal background task, checks mission counts and loaded playback/chart state, snapshots the scene, and exits. A watchdog makes failure explicit. It does not replace interactive testing or physical validation.

See [native validation](../native/docs/VALIDATION.md), [model](../native/docs/MODEL.md), and [research answers](../native/docs/RESEARCH_ANSWERS.md). No measured Argonne requirements or actual flight logs were supplied. These checks establish software behavior and limited numerical consistency, not field accuracy or certification.
