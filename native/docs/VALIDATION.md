# Validation record

Validated in the Linux cloud workspace using GCC 14.2.0, C++17, GNU Make, and Python 3.12. No external C++ libraries are required.

| Check | Command or method | Observed result |
| --- | --- | --- |
| Full application build | `make -j4 all` | Passed with warning flags enabled |
| C++ verification | `make test` | 28/28 test groups passed |
| Planner reference comparison | Every 3×3 obstacle map and endpoint pair against independent BFS | 41,472 comparisons passed |
| Memory and undefined behavior | `make -j4 sanitize` | All 28 groups and CLI checks passed under ASan/UBSan |
| Command-line behavior | `tests/test_cli.py` | Invalid-input rejection, strict JSON, files, telemetry, Monte Carlo and weather rejection passed |
| Analytic physics | Vacuum parcel time/range, calm hover, energy balance, acoustic scaling | Passed numerical tolerances in tests |
| Coupled missions | Default fleet, busy hub, obstacle-aware landing, energy-limited launch, timeouts | Passed |
| Busy hub regression | 3 aircraft, 30 deliveries, seed42, 1800s horizon | All 30 returned; zero separation-threshold events |
| Statistical experiments | 13 scenarios × 30 seeds, plus two deterministic timestep checks | 392 scenario executions; complete manifest and raw results |
| Reusable setup | `/workspace/cloud-setup/embedded-drone/install-full.sh` | Rebuilt/reused outputs and verified six returns, six accepted parcels and zero separation events |
| Visual Studio project integrity | Parsed both project files and resolved every source/header reference | 13 references and four C++17 configurations in each project; Windows compilation unrun |

No tests were disabled or marked as expected failures. The original unreachable-goal planner hang is fixed and covered by regression tests. Scenario rejection and parcel misses in the research runs remain visible as outcomes; these are not relabeled as passing deliveries.

The numerical convergence comparison is a calm, noiseless scenario. The two timesteps produce slightly different release/event timing; the measured answers document that sensitivity rather than claiming exact continuum convergence.

This record establishes software and model checks. Real-aircraft calibration, field validation, six-degree-of-freedom attitude/motor modeling, and measured acoustic validation remain outside the evidence provided. See the model document for assumptions.
