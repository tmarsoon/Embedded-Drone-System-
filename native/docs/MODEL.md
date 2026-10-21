# Model, units, and interpretation

## Scenario and coordinates

Positions use a local east/north/up frame in metres, velocity is metres/second, mass is kilograms, thrust is newtons, and energy is watt-hours. Ground is a flat plane at z=0. The baseline has three aircraft, six destinations on a 150m-radius ring, 0.3kg parcels, a 2kg empty aircraft, 160Wh batteries, and a 0.1s integration timestep. Lowest cruise altitude is 40m; successive aircraft use lanes 12m higher. Individual hub pads are spaced 20m apart.

Each job ascends, departs the shared hub corridor, flies to a forecast-compensated release point, releases its parcel, returns to a holding point, obtains hub access, approaches horizontally at cruise altitude, and descends to its pad. Aircraft keep their remaining battery across jobs. A returned aircraft is not automatically recharged. Mission completion requires parcel impact and return to the hub; being inside the acceptance radius is a separate success measure.

FIFO job assignment is a reproducible baseline, not an optimal assignment algorithm. Waiting arrivals have corridor priority over new launches. Before a new ascent or descent, a geometric reservation checks other aircraft's estimated positions and remaining route segments against that pad's vertical column. Its protected radius adds `max(2m, 3*gpsSigma)` to the separation threshold. Conflicting access is deferred without discarding the job. Altitude lanes and protected hub columns reduce conflicts; actual closest separation is estimated along relative linear motion within each integration step. A zero-event run is an observed result, not a collision-avoidance proof. The monitor excludes parked aircraft. Vehicles are points, with a configurable 8m separation criterion; rotor wake interactions and curved swept-body collision checks are absent.

## Translational dynamics and parcel release

For mass m, velocity v, wind w, thrust T and effective drag area CdA:

```text
m dv/dt = T + m (0,0,-g) - 0.5 rho CdA |v-w| (v-w)
dp/dt = v
g = 9.80665 m/s², rho = 1.225 kg/m³
```

Aircraft and payload states use fourth-order Runge–Kutta, with commanded thrust and sampled wind fixed during each timestep; velocity-dependent drag is recomputed at each RK4 stage. A payload uses T=0 and receives the aircraft's actual velocity at release. Payload mass is removed from the aircraft immediately, and the controller's thrust feedforward uses the new mass. Ground impact is located by bisection inside the timestep; subsequent payload updates leave its impact fixed. There is no parachute, lift, terrain, bounce, or package rotation model.

The release planner integrates the parcel model under the forecast wind and offsets the delivery point by the predicted horizontal drift. Its nominal horizontal release velocity is zero; the controller approaches this point and waits for estimated speed below 0.8m/s. Actual gusts, altitude-dependent wind during the fall, residual release velocity and estimation error can cause misses. The planner does not receive future gust realizations or payload impact truth.

The aircraft model aggregates rotor thrust as an immediately available vector capped by the configured thrust magnitude. It does not simulate six-degree-of-freedom attitude, rotor response, propeller maps, flexible structures, or aerodynamic ground effect. Its force/mass limit and controller therefore represent a translational research model rather than a hardware autopilot.

## Wind, measurements, and estimation

Mean wind direction is a vector angle counterclockwise from +x, not a meteorological direction-of-origin. Mean speed is multiplied by `1 + windShear * max(z,0)`. Forecast speed can have a configured bias and excludes realized gusts.

Each gust component follows an Ornstein–Uhlenbeck process. The update is exact for a timestep dt:

```text
rho_t = exp(-dt/tau)
gust_next = rho_t * gust + sigma * sqrt(1-rho_t²) * N(0,1)
```

Horizontal components have the configured stationary standard deviation; vertical standard deviation is one fifth of that value. Initial gusts are sampled from the stationary distribution. Gusts are spatially shared across the fleet at a given time; only mean shear depends on altitude. This preserves within-run weather dependence but does not model a spatial turbulence field or building wakes.

The navigation filter has position and velocity states for each axis. Noisy accelerometer measurements drive prediction; noisy GPS updates arrive approximately every 0.2s and may drop out. Measurements are synthetically generated from the physical state; the controller receives the filter estimate, not truth position or velocity. `max_navigation_error_m` measures estimated-versus-actual position error, not route tracking error. Covariances use Joseph-form correction with small variance floors. Accelerometer noise is independent Gaussian sample noise plus a small process-model variance. Orientation, gravity-vector estimation, magnetic interference, multipath bias, and persistent sensor drift are not represented.

The simulator has separate seeded wind and per-aircraft sensor streams. The same seed, binary and inputs reproduce the same run. Standard-library random-normal implementations can differ between toolchains, so bit-identical results across compilers/platforms are not promised. The manifest identifies the actual binary and source hashes.

## Routing and constrained control

A* operates on a four-neighbour grid, records the best g-cost, and reconstructs owned path values. Blocked and unreachable endpoints return an empty path; malformed grids throw. The legacy path API returns goal-to-start order, and reconstructed `Node::parent` values are null.

Flight routes inflate no-fly columns by a clearance margin and grid-cell allowance. Collinear points are reduced to route corners. No-fly restrictions extend vertically; transit and hub-approach paths must have valid routes. This is a grid route, not a dynamically optimal or globally minimum-energy trajectory. The simulation checks actual flown samples in regression cases; arbitrary gusts and sensor failures can still cause tracking deviations.

The controller turns position error into a desired velocity, limits speed with a braking-distance envelope, and computes a bounded acceleration using velocity error and a bounded integral term. Gravity and forecast aerodynamic drag are feedforward terms. Integral accumulation freezes under thrust saturation. Gains are fixed; this model does not claim adaptive-gain stability guarantees.

## Battery and operating policy

Electrical power combines momentum-theory induced power with assumed avionics/profile and parasite terms:

```text
P = 15W avionics + 20W running rotors
  + |T|^(3/2) / (eta * sqrt(2 rho A_rotor))
  + 0.5 rho CdA |v-w|³ / eta
energy_increment_Wh = P * dt / 3600
```

The 15W/20W constants and efficiency are scenario assumptions. Power is evaluated once per control step. Battery state is energy accounting, not a voltage/SOC/temperature/electrochemical model. Motors are unpowered after stored energy reaches zero; within-step depletion is only resolved at the timestep scale.

Preflight energy forecasts use planned out-and-back route lengths, climb/descent time, a 15s service allowance, and a 1.3 uncertainty multiplier. Required energy plus reserve must fit in the current battery. Forecast power is not a formal upper bound on every wind realization or hub queue. Reserve checks during outbound flight can trigger a return with the parcel still attached. Weather, thrust-margin, energy and route rejections are explicit outcomes. Return routes that cannot be formed trigger a reported local emergency landing rather than a fabricated completed mission.

## Acoustic model

The noise model estimates an A-weighted source level at 1m relative to 250W, applies power scaling and spherical spreading, and includes a simple distance absorption term as implemented in `noiseDb`. Multiple aircraft are combined in linear intensity, never by adding dB directly. Reported metrics are maximum instantaneous fleet level, equivalent level LAeq over the scenario duration, and sound exposure level SEL relative to 1s.

The baseline source is 78dB at 1m and the observer is at (40,20,1.5)m. These are illustrative inputs, not measured drone acoustics. There is no frequency spectrum, tonal directivity, terrain reflection, ambient noise, weather-dependent propagation or regulatory compliance model. A different duration changes LAeq's averaging window, so compare SEL when comparing total exposure across unequal-duration scenarios.

## Monte Carlo and confidence

The experiment runner uses seed42 onward, records complete input parameters and individual-run JSON, and hashes the binary and source files. Pooled parcel percentiles are descriptive. Parcels from the same run share weather and are correlated; the primary error confidence interval uses one mean impact error per independent run. Runs with no impacts have no defined mean impact error and are omitted from that error statistic, while unsuccessful requests remain in success-rate denominators.

Intervals use the normal approximation for a mean, not a prediction interval or a small-sample guarantee. Success is also summarized by per-run fractions. Zero observed failures gives zero observed variance; it does not prove zero failure probability. Results are conditional on the configured distributions, geometry and policies. Changing integration timestep changes stochastic draw timing, so numerical convergence comparisons use a separate noiseless, calm scenario.

## What is verified and what still needs data

Automated tests check analytic vacuum ballistic motion, exact calm hover, energy accounting, wind and noise limits, filtering behavior, route correctness, fleet return, policy rejects, timeout accounting and deterministic timestep convergence. These establish numerical and software behavior under the model.

A real-aircraft study still needs measured mass/drag/rotor maps, validated wind fields, sensor calibration, battery discharge data, acoustic measurements, actual hub geometry, operating constraints, and comparison with flight logs. No Argonne hardware, unpublished requirements or validation data were provided. The included research questions and answers identify plausible experiments from the user's scope rather than claiming institutional endorsement.
