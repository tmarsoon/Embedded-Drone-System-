#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::ostringstream detail;
        detail << message << ": actual=" << actual << ", expected=" << expected
               << ", tolerance=" << tolerance;
        throw std::runtime_error(detail.str());
    }
}

void nearVector(sim::Vec3 actual, sim::Vec3 expected, double tolerance,
                const std::string& message) {
    near(actual.x, expected.x, tolerance, message + " x");
    near(actual.y, expected.y, tolerance, message + " y");
    near(actual.z, expected.z, tolerance, message + " z");
}

template <class Exception = std::exception, class Function>
void throws(Function operation, const std::string& message) {
    try {
        operation();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

int bfsDistance(const std::vector<std::vector<int>>& grid, int sx, int sy, int gx, int gy) {
    if (grid[sx][sy] || grid[gx][gy]) return -1;
    int distance[3][3];
    for (auto& row : distance) std::fill(std::begin(row), std::end(row), -1);
    std::queue<std::pair<int, int>> queue;
    distance[sx][sy] = 0;
    queue.emplace(sx, sy);
    constexpr int directions[][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop();
        if (x == gx && y == gy) return distance[x][y];
        for (const auto& direction : directions) {
            const int nx = x + direction[0], ny = y + direction[1];
            if (nx >= 0 && nx < 3 && ny >= 0 && ny < 3 && !grid[nx][ny]
                && distance[nx][ny] < 0) {
                distance[nx][ny] = distance[x][y] + 1;
                queue.emplace(nx, ny);
            }
        }
    }
    return -1;
}

void exhaustivePlanner() {
    std::size_t comparisons = 0;
    for (unsigned mask = 0; mask < 512; ++mask) {
        std::vector<std::vector<int>> grid(3, std::vector<int>(3, 0));
        for (int x = 0; x < 3; ++x)
            for (int y = 0; y < 3; ++y) grid[x][y] = (mask >> (3*x+y)) & 1U;
        AStarPlanner planner(3, 3, grid);
        for (int start = 0; start < 9; ++start) {
            for (int goal = 0; goal < 9; ++goal) {
                const int sx = start/3, sy = start%3, gx = goal/3, gy = goal%3;
                const int expected = bfsDistance(grid, sx, sy, gx, gy);
                const auto path = planner.plan(sx, sy, gx, gy);
                ++comparisons;
                if (expected < 0) {
                    require(path.empty(), "A* returned a path to an unreachable goal");
                    continue;
                }
                require(path.size() == static_cast<std::size_t>(expected + 1),
                        "A* path differs from independent BFS shortest distance");
                require(path.front().x == gx && path.front().y == gy
                        && path.back().x == sx && path.back().y == sy,
                        "A* goal-to-start endpoint contract");
                for (std::size_t i = 0; i < path.size(); ++i) {
                    const Node& node = path[i];
                    require(node.x >= 0 && node.x < 3 && node.y >= 0 && node.y < 3,
                            "A* path leaves grid");
                    require(!grid[node.x][node.y], "A* path crosses an obstacle");
                    require(node.parent == nullptr, "returned path contains a raw parent reference");
                    near(node.cost, expected - static_cast<double>(i), 0.0,
                         "A* reports true cost from start");
                    if (i) require(std::abs(node.x-path[i-1].x)+std::abs(node.y-path[i-1].y)==1,
                                   "A* path contains a non-adjacent step");
                }
            }
        }
    }
    require(comparisons == 41472, "exhaustive planner coverage count");
}

void dynamicPlanner() {
    std::vector<std::vector<int>> grid(3, std::vector<int>(3));
    DynamicAStarPlanner planner(3, 3, grid);
    planner.addDynamicObstacle(1, 0);
    require(planner.plan(0, 0, 2, 0).size() == 5, "dynamic detour");
    planner.moveObstacle(1, 0, 1, 1);
    require(planner.plan(0, 0, 2, 0).size() == 3, "moving obstacle clears direct path");
    planner.addDynamicObstacle(0, 0);
    require(planner.plan(0, 0, 2, 0).empty(), "blocked start rejected");
    planner.removeDynamicObstacle(0, 0);
    require(!planner.plan(0, 0, 2, 0).empty(), "removed obstacle restores path");
    throws<std::out_of_range>([&] { planner.moveObstacle(1, 1, 9, 9); },
                              "out-of-bounds obstacle accepted");
    require(planner.plan(1, 1, 2, 0).empty(), "invalid move changed the old obstacle");
}

void plannerValidation() {
    throws<std::invalid_argument>([] { AStarPlanner p(0, 0, {}); }, "empty grid accepted");
    throws<std::invalid_argument>([] { AStarPlanner p(2, 2, {{0}, {0, 0}}); },
                                 "ragged grid accepted");
    AStarPlanner planner(2, 2, {{0, 0}, {1, 1}});
    require(planner.plan(-1, 0, 0, 0).empty(), "out-of-grid start accepted");
    require(planner.plan(0, 0, 2, 0).empty(), "out-of-grid goal accepted");
    require(planner.plan(0, 0, 1, 1).empty(), "unreachable cycle did not terminate");
}

sim::Config noiselessConfig() {
    sim::Config config;
    config.windSpeed = 0.0;
    config.gustSigma = 0.0;
    config.gpsSigma = 0.0;
    config.accelerometerSigma = 0.0;
    config.gpsDropoutProbability = 0.0;
    return config;
}

void vehiclePhysics() {
    sim::Config config = noiselessConfig();
    config.vehicleDragArea = 0.0;
    sim::VehicleState state;
    state.position = {0, 0, 100};
    state.batteryWh = 100;
    sim::advanceVehicle(state, {}, {}, config.vehicleMass, 0.1, config);
    near(state.position.z, 100.0 - 0.5*sim::gravity*0.01, 1e-9, "freefall position");
    near(state.velocity.z, -sim::gravity*0.1, 1e-9, "freefall velocity");
    near(state.acceleration.z, -sim::gravity, 1e-9, "freefall acceleration");

    state = {};
    state.position = {0, 0, 20};
    state.batteryWh = 100;
    const double power = sim::advanceVehicle(state, {0, 0, config.vehicleMass*sim::gravity},
                                             {}, config.vehicleMass, 1.0, config);
    nearVector(state.position, {0, 0, 20}, 1e-10, "calm hover position");
    nearVector(state.velocity, {}, 1e-10, "calm hover velocity");
    require(power > 0 && std::isfinite(power), "hover requires positive finite power");
    near(state.energyWh, power/3600.0, 1e-10, "power/time energy integration");
    near(state.batteryWh, 100.0-state.energyWh, 1e-10, "battery energy balance");
}

void payloadAnalytic() {
    sim::PayloadState state;
    state.position = {0, 0, 40};
    state.velocity = {5, -2, 0};
    state.dragArea = 0.0;
    for (int i = 0; i < 1000 && !state.impacted; ++i) sim::advancePayload(state, {}, 0.07);
    require(state.impacted, "vacuum payload never reached ground");
    const double exactTime = std::sqrt(80.0/sim::gravity);
    near(state.flightTime, exactTime, 2e-6, "vacuum impact time vs analytic solution");
    nearVector(state.impact, {5*exactTime, -2*exactTime, 0}, 1e-5,
               "vacuum horizontal range vs analytic solution");
    const sim::PayloadState landed = state;
    sim::advancePayload(state, {20, 0, 0}, 2.0);
    nearVector(state.position, landed.position, 0, "landed parcel cannot move");
    near(state.flightTime, landed.flightTime, 0, "landed flight clock cannot advance");
}

void payloadWind() {
    const auto calm = sim::predictPayloadDrift(40, {}, {}, 0.3, 0.018);
    const auto windy = sim::predictPayloadDrift(40, {}, {5, 0, 0}, 0.3, 0.018);
    near(calm.x, 0, 1e-10, "calm vertical drop horizontal range");
    require(windy.x > 0.5, "parcel must drift downwind");
    near(windy.y, 0, 1e-10, "x-only wind creates no crosswind drift");
    near(windy.z, 0, 1e-10, "predictor ends on ground");
    sim::Config config;
    const double light = sim::electricalPower(config.vehicleMass*sim::gravity, 0, config);
    const double loaded = sim::electricalPower((config.vehicleMass+config.payloadMass)*sim::gravity,
                                               0, config);
    require(loaded > light, "carrying payload must increase hover power");
}

void aerodynamicDrag() {
    sim::Config config;
    sim::VehicleState state;
    state.position = {0, 0, 30};
    state.velocity = {5, 0, 0};
    state.batteryWh = 100;
    sim::advanceVehicle(state, {0, 0, config.vehicleMass*sim::gravity}, {},
                        config.vehicleMass, 0.1, config);
    require(state.velocity.x > 0 && state.velocity.x < 5, "drag must oppose relative motion");
    state.velocity = {};
    sim::advanceVehicle(state, {0, 0, config.vehicleMass*sim::gravity}, {5, 0, 0},
                        config.vehicleMass, 0.1, config);
    require(state.velocity.x > 0, "vehicle must accelerate downwind without compensation");
}

void windReproducibility() {
    sim::Config config;
    sim::WindField first(config), second(config);
    config.seed += 1;
    sim::WindField different(config);
    bool seedMatters = false;
    for (int i = 0; i < 100; ++i) {
        first.step(0.1); second.step(0.1); different.step(0.1);
        nearVector(first.at({0, 0, 40}), second.at({0, 0, 40}), 0, "same-seed wind reproducibility");
        seedMatters = seedMatters || sim::norm(first.at({0, 0, 40})-different.at({0, 0, 40})) > 1e-6;
    }
    require(seedMatters, "different seeds must produce different gust histories");
    config.gustSigma = 0;
    config.windSpeed = 3;
    config.windDirectionDegrees = 0;
    config.windShear = 0;
    sim::WindField steady(config);
    steady.step(10);
    nearVector(steady.at({0, 0, 40}), {3, 0, 0}, 1e-12, "zero gust reproduces mean wind");
    nearVector(steady.forecastAt({0, 0, 40}), {3, 0, 0}, 1e-12, "unbiased steady forecast");
}

void acousticScaling() {
    sim::Config config;
    near(sim::noiseDb(400, 10, config)-sim::noiseDb(200, 10, config),
         10*std::log10(2.0), 1e-10, "doubling acoustic power adds 3 dB");
    near(sim::noiseDb(200, 10, config)-sim::noiseDb(200, 20, config),
         20*std::log10(2.0), 0.2, "distance doubling gives approximately 6 dB attenuation");
}

void kalmanExactMotion() {
    const auto config = noiselessConfig();
    sim::NavigationFilter filter({}, config, 91);
    sim::VehicleState truth;
    const sim::Vec3 acceleration{0.2, -0.1, 0.05};
    for (int i = 1; i <= 100; ++i) {
        const double time = i*0.1;
        truth.position = acceleration*(0.5*time*time);
        truth.velocity = acceleration*time;
        truth.acceleration = acceleration;
        const auto estimate = filter.update(truth, 0.1);
        nearVector(estimate.position, truth.position, 1e-5, "noiseless Kalman accelerated position");
        nearVector(estimate.velocity, truth.velocity, 1e-5, "noiseless Kalman accelerated velocity");
    }
}

void kalmanDropout() {
    auto config = noiselessConfig();
    config.gpsDropoutProbability = 1.0;
    sim::NavigationFilter filter({}, config, 91);
    sim::VehicleState truth;
    for (int i = 1; i <= 20; ++i) {
        const double time = i*0.1;
        truth.position = {0.05*time*time, 0, 0};
        truth.velocity = {0.1*time, 0, 0};
        truth.acceleration = {0.1, 0, 0};
        filter.update(truth, 0.1);
    }
    const auto estimate = filter.state();
    require(!estimate.gpsAvailable, "complete GPS dropout must be reported");
    nearVector(estimate.position, truth.position, 1e-6, "short inertial-only position prediction");
}

void kalmanNoiseAndReproducibility() {
    auto config = noiselessConfig();
    config.gpsSigma = 2.0;
    config.accelerometerSigma = 0.02;
    sim::NavigationFilter filter({}, config, 132), same({}, config, 132);
    sim::VehicleState truth;
    double squaredError = 0;
    int components = 0;
    for (int i = 0; i < 5000; ++i) {
        const auto estimate = filter.update(truth, 0.1);
        nearVector(estimate.position, same.update(truth, 0.1).position, 0,
                   "same-seed sensor/filter reproducibility");
        if (i >= 1000) {
            const auto error = estimate.position-truth.position;
            squaredError += error.x*error.x+error.y*error.y+error.z*error.z;
            components += 3;
        }
    }
    const double rmse = std::sqrt(squaredError/components);
    require(rmse < config.gpsSigma*0.75, "Kalman filter must reduce stationary GPS noise RMSE");
}

void controllerBehavior() {
    auto config = noiselessConfig();
    sim::FlightController controller(config);
    sim::Estimate estimate;
    estimate.position = {0, 0, 40};
    const auto hover = controller.command(estimate.position, estimate, config.vehicleMass, {}, 0.1);
    nearVector(hover.force, {0, 0, config.vehicleMass*sim::gravity}, 1e-8,
               "controller hover gravity compensation");
    config.maxThrust = 5;
    sim::FlightController limited(config);
    const auto saturated = limited.command({100, 100, 100}, estimate, config.vehicleMass, {}, 0.1);
    require(sim::norm(saturated.force) <= config.maxThrust+1e-10, "thrust exceeds physical limit");
    require(saturated.saturated, "thrust saturation must be reported");
    throws<std::invalid_argument>([&] { controller.command({}, estimate, 2, {}, 0); },
                                 "controller accepts zero timestep");
}

void metricStatistics() {
    const auto summary = sim::summarizeMetric({1, 2, 3, 4});
    require(summary.count == 4, "metric sample count");
    near(summary.mean, 2.5, 1e-12, "sample mean");
    near(summary.stddev, std::sqrt(5.0/3.0), 1e-12, "sample standard deviation");
    near(summary.p50, 2.5, 1e-12, "interpolated median");
    near(summary.p95, 3.85, 1e-12, "interpolated 95th percentile");
    constexpr double normal95 = 1.959963984540054;
    near(summary.ci95Low, 2.5-normal95*std::sqrt(5.0/3.0)/2, 1e-10, "normal approximate CI lower");
    near(summary.ci95High, 2.5+normal95*std::sqrt(5.0/3.0)/2, 1e-10, "normal approximate CI upper");
    require(sim::summarizeMetric({}).count == 0, "empty metric has no samples");
    throws<std::invalid_argument>([] { sim::summarizeMetric({1, std::numeric_limits<double>::quiet_NaN()}); },
                                 "nonfinite metric data accepted");
}

void aggregateStatistics() {
    sim::SimulationResult first, second;
    first.requested = second.requested = 2;
    first.delivered = second.delivered = 1;
    first.energyWh = 10; second.energyWh = 30;
    first.duration = 20; second.duration = 40;
    sim::MissionResult delivered, absent;
    delivered.delivered = true; delivered.impactError = 1;
    absent.delivered = false; absent.impactError = 100;
    first.missions = {delivered, absent};
    delivered.impactError = 3;
    second.missions = {delivered, absent};
    const auto summary = sim::summarizeRuns({first, second});
    require(summary.runs == 2 && summary.requested == 4 && summary.delivered == 2,
            "batch counts preserve incomplete missions");
    require(summary.deliveryError.count == 2, "undelivered parcel included in impact statistics");
    near(summary.deliveryError.mean, 2, 1e-12, "pooled delivered parcel error");
    near(summary.energyWh.mean, 20, 1e-12, "run-level energy statistics");
}

void hubScheduling() {
    sim::HubScheduler hub(3);
    require(hub.nextJob() == 0 && hub.nextJob() == 1 && hub.nextJob() == 2,
            "hub job order must be FIFO");
    require(hub.empty() && !hub.nextJob(), "exhausted job queue");
    require(hub.acquire(1) && hub.acquire(1), "same owner token is idempotent");
    require(!hub.acquire(2) && hub.owner() == 1, "two aircraft acquired hub corridor");
    throws<std::logic_error>([&] { hub.release(2); }, "nonowner released hub token");
    hub.release(1);
    require(!hub.owner() && hub.acquire(2), "hub token not reusable");
}

void energyPolicy() {
    sim::Config config;
    const auto normal = sim::assessMissionEnergy(config.batteryWh, 300, 40,
                                                config.vehicleMass+config.payloadMass, config);
    require(normal.sufficient && normal.requiredWh > 0,
            "default battery should cover representative local mission plus reserve");
    const auto empty = sim::assessMissionEnergy(0.1, 300, 40,
                                               config.vehicleMass+config.payloadMass, config);
    require(!empty.sufficient, "empty battery authorized for dispatch");
    const auto further = sim::assessMissionEnergy(config.batteryWh, 600, 40,
                                                 config.vehicleMass+config.payloadMass, config);
    require(further.requiredWh > normal.requiredWh, "longer route must require more energy");
}

void publicValidation() {
    auto config = noiselessConfig();
    config.dt = 0;
    throws<std::invalid_argument>([&] { sim::validateConfig(config); }, "zero timestep configuration accepted");
    config = noiselessConfig(); config.drones = -1;
    throws<std::invalid_argument>([&] { sim::validateConfig(config); }, "negative aircraft count accepted");
    config = noiselessConfig(); config.gpsDropoutProbability = 1.1;
    throws<std::invalid_argument>([&] { sim::validateConfig(config); }, "invalid GPS probability accepted");
    config = noiselessConfig(); config.windSpeed = std::numeric_limits<double>::quiet_NaN();
    throws<std::invalid_argument>([&] { sim::validateConfig(config); }, "NaN configuration accepted");
    sim::PayloadState payload;
    payload.position.z = 40;
    throws<std::invalid_argument>([&] { sim::advancePayload(payload, {}, 0); }, "zero payload timestep accepted");
    config = noiselessConfig();
    sim::NavigationFilter filter({}, config, 1);
    throws<std::invalid_argument>([&] { filter.update({}, 0); }, "zero filter timestep accepted");
}

void jsonAndCsv() {
    sim::SimulationResult result;
    const auto json = sim::resultJson(result, sim::Config{});
    require(!json.empty() && json.front() == '{' && json.back() == '}', "JSON object framing");
    require(json.find("null") != std::string::npos, "undefined physical metrics must use JSON null");
    require(json.find(":inf") == std::string::npos && json.find(":nan") == std::string::npos,
            "JSON contains nonstandard nonfinite literals");
    sim::TelemetrySample sample;
    sample.time = 0.1; sample.droneId = 2; sample.phase = "outbound";
    sample.position = {1, 2, 3}; sample.estimatedPosition = {1.1, 2.1, 3.1};
    sample.batteryWh = 99; sample.payloadAttached = true;
    std::ostringstream csv;
    sim::writeTraceCsv(csv, {sample});
    const auto text = csv.str();
    require(text.find("outbound") != std::string::npos, "CSV lost mission phase");
    require(std::count(text.begin(), text.end(), '\n') >= 2,
            "CSV must include header and sample");
}

void defaultMission() {
    const sim::Config config;
    const auto result = sim::runSimulation(config, true);
    require(!result.timedOut, "default hub/spoke mission timed out");
    require(result.requested == 6 && result.completed == 6 && result.delivered == 6,
            "default fleet must deliver all parcels and return to hub");
    require(result.rejected == 0 && result.emergencyReturns == 0, "default mission rejected or aborted");
    require(result.separationEvents == 0, "default fleet violated separation policy");
    require(result.minimumSeparation >= config.minSeparation,
            "default fleet minimum separation below policy");
    require(result.energyWh > 0 && result.duration > 0, "completed mission has no physical cost");
    require(std::isfinite(result.maxNoiseDb) && std::isfinite(result.noiseLaeqDb),
            "active fleet missing acoustic metrics");
    require(result.noiseLaeqDb <= result.maxNoiseDb+1e-8, "average sound exceeds peak sound");
    require(!result.trace.empty(), "requested telemetry trace was not produced");
    bool carried = false, detachedAirborne = false;
    for (const auto& sample : result.trace) {
        require(sim::finite(sample.position) && sim::finite(sample.estimatedPosition),
                "telemetry includes nonfinite navigation state");
        require(sample.batteryWh >= 0, "battery went negative");
        carried = carried || sample.payloadAttached;
        detachedAirborne = detachedAirborne || (!sample.payloadAttached && sample.position.z > 10);
    }
    require(carried && detachedAirborne, "payload release did not change vehicle mass state");
    require(result.missions.size() == 6, "missing mission outcome records");
    for (const auto& mission : result.missions) {
        require(mission.completed && mission.delivered && std::isfinite(mission.impactError),
                "completed mission has no impact record");
        require(mission.impactAt >= mission.releasedAt && mission.completedAt >= mission.launchedAt,
                "mission event chronology reversed");
        require(mission.energyWh > 0, "mission energy was not attributed");
    }
}

void missionReproducibility() {
    sim::Config config;
    config.drones = 1; config.deliveries = 1;
    const auto first = sim::runSimulation(config);
    const auto second = sim::runSimulation(config);
    require(first.completed == 1 && second.completed == 1, "reproducibility scenario failed");
    near(first.energyWh, second.energyWh, 0, "same-seed mission energy");
    near(first.duration, second.duration, 0, "same-seed mission duration");
    near(first.missions.front().impactError, second.missions.front().impactError, 0,
         "same-seed payload impact");
}

void preflightRejection() {
    sim::Config config;
    config.windSpeed = 20; config.gustSigma = 0;
    auto result = sim::runSimulation(config);
    require(result.rejected == config.deliveries && result.completed == 0 && result.delivered == 0,
            "unsafe forecast weather must reject dispatch without fake deliveries");
    require(!result.timedOut && result.energyWh == 0, "weather rejection flew aircraft");
    config = sim::Config{}; config.batteryWh = 0.1;
    result = sim::runSimulation(config);
    require(result.rejected == config.deliveries && result.completed == 0 && result.delivered == 0,
            "insufficient energy must reject dispatch");
    require(!result.timedOut, "energy rejection stalled queue");
}

void noFlyRouting() {
    auto config = noiselessConfig();
    config.drones = 1; config.deliveries = 1; config.spokeRadius = 100;
    config.noFlyZones.push_back({30, -15, 70, 15});
    const auto result = sim::runSimulation(config, true);
    require(result.completed == 1 && !result.timedOut, "A* detour did not finish mission");
    for (const auto& sample : result.trace) {
        if (sample.position.z > 1) {
            require(!(sample.position.x > 30 && sample.position.x < 70
                      && sample.position.y > -15 && sample.position.y < 15),
                    "simulated trajectory crossed configured no-fly zone");
        }
    }
}

void noFlyLandingCorridor() {
    auto config = noiselessConfig();
    config.drones = 1; config.deliveries = 1;
    // The old direct descent from (0,-50,altitude) to the hub would cross this
    // rectangle even after a correct obstacle-avoiding outbound/return route.
    config.noFlyZones.push_back({-4, -32, 4, -18});
    const auto result = sim::runSimulation(config, true);
    require(result.completed == 1 && !result.timedOut,
            "landing corridor obstacle prevented safe rerouting and completion");
    bool observedLanding = false;
    for (const auto& sample : result.trace) {
        observedLanding = observedLanding || sample.phase == "landing";
        if (sample.position.z > 0.25) {
            require(!(sample.position.x > -4 && sample.position.x < 4
                      && sample.position.y > -32 && sample.position.y < -18),
                    "aircraft crossed no-fly rectangle during approach/descent");
        }
    }
    require(observedLanding, "landing route regression did not exercise landing phase");
}

void detourEnergyPreflight() {
    auto config = noiselessConfig();
    config.drones = 1; config.deliveries = 1; config.batteryWh = 10;
    // The radial straight-line estimate is too optimistic: the route must go
    // around a 400 m wall and still budget its return, hub approach and reserve.
    config.noFlyZones.push_back({40, -200, 80, 200});
    const auto result = sim::runSimulation(config, true);
    require(result.rejected == 1 && result.completed == 0 && result.delivered == 0,
            "battery policy dispatched an unaffordable obstacle detour");
    require(result.missions.size() == 1 && result.missions.front().outcome == "rejected_energy",
            "detour energy rejection lost its explicit reason");
    require(!result.timedOut && result.energyWh == 0 && result.trace.empty(),
            "unaffordable detour consumed flight energy before rejection");
}

void timeoutAccounting() {
    sim::Config config;
    config.maxTime = 3.0;
    const auto result = sim::runSimulation(config, true);
    require(result.timedOut && result.completed == 0 && result.delivered == 0,
            "short run reported a false delivery/completion");
    require(result.energyWh > 0, "active aircraft did not consume physical energy");
    require(result.missions.size() == static_cast<std::size_t>(config.deliveries),
            "timeout omitted requested but not-yet-dispatched jobs");
    double attributedEnergy = 0;
    int pending = 0, active = 0;
    for (const auto& mission : result.missions) {
        attributedEnergy += mission.energyWh;
        require(!mission.completed && !mission.delivered, "unfinished mission marked completed");
        if (mission.outcome == "not_dispatched_timeout") ++pending;
        if (mission.outcome == "simulation_timeout") {
            ++active;
            require(mission.energyWh > 0, "airborne timed-out mission lost its consumed energy");
        }
    }
    require(pending > 0 && active > 0, "timeout regression did not cover active and queued jobs");
    near(attributedEnergy, result.energyWh, 1e-9,
         "mission energy attribution does not reconcile with fleet electrical energy");
}

void busyHubColumnSeparation() {
    sim::Config config;
    config.drones = 3;
    config.deliveries = 30;
    config.maxTime = 1800;
    config.seed = 42;
    const auto result = sim::runSimulation(config);
    require(!result.timedOut && result.completed == config.deliveries
            && result.delivered == config.deliveries,
            "busy hub did not deliver all parcels and return every mission");
    require(result.rejected == 0 && result.emergencyReturns == 0,
            "busy hub regression unexpectedly rejected or aborted jobs");
    require(result.separationEvents == 0,
            "returning aircraft crossed another aircraft's ascent/descent column");
    require(result.minimumSeparation >= config.minSeparation,
            "busy hub violated minimum aircraft separation");
}

void timestepConvergence() {
    auto config = noiselessConfig();
    config.drones = 1; config.deliveries = 1; config.spokeRadius = 100;
    const auto coarse = sim::runSimulation(config);
    config.dt = 0.05;
    const auto fine = sim::runSimulation(config);
    require(coarse.completed == 1 && fine.completed == 1, "timestep comparison missions failed");
    near(coarse.energyWh, fine.energyWh, 0.05*fine.energyWh,
         "halving timestep changes energy by more than 5 percent");
    near(coarse.duration, fine.duration, 0.1*fine.duration,
         "halving timestep changes completion time by more than 10 percent");
    near(coarse.missions.front().impactError, fine.missions.front().impactError, 0.5,
         "halving timestep changes drop error by more than half a meter");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"A* exhaustive comparison with independent BFS (41,472 cases)", exhaustivePlanner},
        {"dynamic obstacles and atomic invalid movement", dynamicPlanner},
        {"planner validation and unreachable-cycle termination", plannerValidation},
        {"vehicle gravity, hover and electrical energy balance", vehiclePhysics},
        {"payload analytic vacuum impact and landed-state invariance", payloadAnalytic},
        {"payload downwind drift and load-dependent power", payloadWind},
        {"aerodynamic drag opposes relative airflow", aerodynamicDrag},
        {"correlated wind reproducibility and steady limiting case", windReproducibility},
        {"acoustic power and distance scaling", acousticScaling},
        {"Kalman filter exact accelerated limiting case", kalmanExactMotion},
        {"Kalman inertial prediction through GPS dropout", kalmanDropout},
        {"Kalman noise reduction and seeded reproducibility", kalmanNoiseAndReproducibility},
        {"controller gravity compensation, saturation and validation", controllerBehavior},
        {"sample statistics, quantiles, confidence interval and invalid data", metricStatistics},
        {"batch statistics exclude undelivered parcels", aggregateStatistics},
        {"hub FIFO dispatch and exclusive corridor ownership", hubScheduling},
        {"mission energy requirement and reserve policy", energyPolicy},
        {"invalid public inputs rejected", publicValidation},
        {"JSON null metrics and telemetry CSV output", jsonAndCsv},
        {"default fleet delivery, return, separation and payload release", defaultMission},
        {"full mission seeded reproducibility", missionReproducibility},
        {"weather and battery preflight rejection", preflightRejection},
        {"A* no-fly route obeyed by simulated vehicle", noFlyRouting},
        {"no-fly zone respected during landing corridor approach", noFlyLandingCorridor},
        {"preflight reserve accounts for actual obstacle detour length", detourEnergyPreflight},
        {"active and queued timeout outcomes retain electrical energy", timeoutAccounting},
        {"busy hub ascent/descent columns preserve fleet separation", busyHubColumnSeparation},
        {"mission convergence when timestep is halved", timestepConvergence},
    };
    int failed = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        } catch (...) {
            ++failed;
            std::cerr << "FAIL " << name << ": unknown exception\n";
        }
    }
    std::cout << tests.size()-failed << "/" << tests.size() << " test groups passed\n";
    return failed ? 1 : 0;
}
