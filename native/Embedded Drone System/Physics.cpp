#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
namespace {
void positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
}
void nonnegative(double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
    }
}
Vec3 dragAcceleration(Vec3 velocity, Vec3 wind, double mass, double dragArea) {
    const Vec3 relative = velocity - wind;
    return relative * (-0.5 * airDensity * dragArea * norm(relative) / mass);
}
struct KinematicState { Vec3 position, velocity; };
struct Derivative { Vec3 position, velocity; };

KinematicState integrate(KinematicState state, Vec3 appliedAcceleration,
                         Vec3 wind, double mass, double dragArea, double dt) {
    const auto derivative = [&](const KinematicState& s) {
        return Derivative{s.velocity, appliedAcceleration
            + dragAcceleration(s.velocity, wind, mass, dragArea)};
    };
    const auto displaced = [&](Derivative d, double scale) {
        return KinematicState{state.position + d.position * scale,
                              state.velocity + d.velocity * scale};
    };
    const Derivative k1 = derivative(state);
    const Derivative k2 = derivative(displaced(k1, 0.5 * dt));
    const Derivative k3 = derivative(displaced(k2, 0.5 * dt));
    const Derivative k4 = derivative(displaced(k3, dt));
    state.position += (k1.position + k2.position * 2.0 + k3.position * 2.0
                       + k4.position) * (dt / 6.0);
    state.velocity += (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0
                       + k4.velocity) * (dt / 6.0);
    if (!finite(state.position) || !finite(state.velocity)) {
        throw std::runtime_error("dynamics became non-finite; reduce timestep or forcing");
    }
    return state;
}
Vec3 meanWind(const Config& config, double height, bool forecast) {
    const double angle = config.windDirectionDegrees * pi / 180.0;
    const double speed = std::max(0.0, config.windSpeed
        + (forecast ? config.windForecastBias : 0.0));
    const double shear = 1.0 + config.windShear * std::max(0.0, height);
    return {speed * std::cos(angle) * shear,
            speed * std::sin(angle) * shear, 0.0};
}
} // namespace

double norm(Vec3 value) {
    return std::hypot(std::hypot(value.x, value.y), value.z);
}
double horizontalNorm(Vec3 value) { return std::hypot(value.x, value.y); }
bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

WindField::WindField(const Config& config)
    : config_(config), random_(config.seed ^ 0x6a09e667U) {
    nonnegative(config_.windSpeed, "wind speed");
    nonnegative(config_.gustSigma, "gust sigma");
    positive(config_.gustCorrelation, "gust correlation");
    nonnegative(config_.windShear, "wind shear");
    if (!std::isfinite(config_.windDirectionDegrees)
        || !std::isfinite(config_.windForecastBias)) {
        throw std::invalid_argument("wind direction and forecast bias must be finite");
    }
    // Start in the stationary distribution rather than suppressing early gusts.
    gust_ = {config_.gustSigma * normal_(random_),
             config_.gustSigma * normal_(random_),
             0.2 * config_.gustSigma * normal_(random_)};
}

void WindField::step(double dt) {
    positive(dt, "wind timestep");
    const double persistence = std::exp(-dt / config_.gustCorrelation);
    const double innovation = config_.gustSigma
        * std::sqrt(-std::expm1(-2.0 * dt / config_.gustCorrelation));
    gust_ = gust_ * persistence
        + Vec3{innovation * normal_(random_), innovation * normal_(random_),
               0.2 * innovation * normal_(random_)};
}

Vec3 WindField::at(Vec3 position) const {
    if (!finite(position)) throw std::invalid_argument("wind position must be finite");
    return meanWind(config_, position.z, false) + gust_;
}

Vec3 WindField::forecastAt(Vec3 position) const {
    if (!finite(position)) throw std::invalid_argument("forecast position must be finite");
    // The controller/release predictor cannot observe the realized random gust.
    return meanWind(config_, position.z, true);
}

double electricalPower(double thrust, double airspeed, const Config& config) {
    nonnegative(thrust, "thrust");
    nonnegative(airspeed, "airspeed");
    positive(config.rotorDiskArea, "rotor disk area");
    positive(config.rotorEfficiency, "rotor efficiency");
    nonnegative(config.vehicleDragArea, "vehicle drag area");
    // Actuator-disk induced power, 20 W running-rotor profile power, 15 W
    // avionics, and parasite-drag power. These are scenario assumptions, not
    // measured motor/propeller maps. Power is electrical watts.
    const double induced = std::pow(thrust, 1.5)
        / std::sqrt(2.0 * airDensity * config.rotorDiskArea)
        / config.rotorEfficiency;
    const double parasite = 0.5 * airDensity * config.vehicleDragArea
        * airspeed * airspeed * airspeed / config.rotorEfficiency;
    const double power = 15.0 + (thrust > 0.0 ? 20.0 : 0.0) + induced + parasite;
    if (!std::isfinite(power)) throw std::overflow_error("electrical power overflow");
    return power;
}

double advanceVehicle(VehicleState& state, Vec3 thrust, Vec3 wind,
                      double mass, double dt, const Config& config) {
    positive(mass, "vehicle mass");
    positive(dt, "vehicle timestep");
    positive(config.maxThrust, "maximum thrust");
    nonnegative(config.vehicleDragArea, "vehicle drag area");
    nonnegative(state.batteryWh, "battery charge");
    nonnegative(state.energyWh, "accumulated energy");
    if (!finite(state.position) || !finite(state.velocity)
        || !finite(thrust) || !finite(wind) || thrust.z < 0.0) {
        throw std::invalid_argument("vehicle state, wind, and upward thrust must be finite");
    }
    const double force = norm(thrust);
    if (force > config.maxThrust) thrust = thrust * (config.maxThrust / force);
    if (state.batteryWh == 0.0) thrust = {};
    const double airspeed = norm(state.velocity - wind);
    const double power = electricalPower(norm(thrust), airspeed, config);
    const Vec3 applied = thrust / mass + Vec3{0.0, 0.0, -gravity};
    KinematicState next = integrate({state.position, state.velocity}, applied,
                                    wind, mass, config.vehicleDragArea, dt);
    if (next.position.z < 0.0) {
        next.position.z = 0.0;
        if (next.velocity.z < 0.0) next.velocity.z = 0.0;
    }
    state.position = next.position;
    state.velocity = next.velocity;
    state.acceleration = applied + dragAcceleration(next.velocity, wind, mass,
                                                     config.vehicleDragArea);
    if (state.position.z == 0.0 && state.acceleration.z < 0.0) {
        state.acceleration.z = 0.0; // Ground normal force.
    }
    const double drawnWh = std::min(state.batteryWh, power * dt / 3600.0);
    state.batteryWh -= drawnWh;
    state.energyWh += drawnWh;
    return power;
}

void advancePayload(PayloadState& state, Vec3 wind, double dt) {
    positive(state.mass, "payload mass");
    nonnegative(state.dragArea, "payload drag area");
    positive(dt, "payload timestep");
    nonnegative(state.flightTime, "payload flight time");
    if (!finite(state.position) || !finite(state.velocity) || !finite(wind)) {
        throw std::invalid_argument("payload state and wind must be finite");
    }
    if (state.impacted) return;
    if (state.position.z <= 0.0) {
        state.position.z = 0.0;
        state.impact = state.position;
        state.impacted = true;
        return;
    }
    const KinematicState previous{state.position, state.velocity};
    const Vec3 applied{0.0, 0.0, -gravity};
    KinematicState next = integrate(previous, applied, wind, state.mass,
                                    state.dragArea, dt);
    double elapsed = dt;
    if (next.position.z <= 0.0) {
        // Solve the landing event inside the integrator step. Linear endpoint
        // interpolation gives a timestep-dependent impact-time bias in vacuum.
        double low = 0.0, high = dt;
        for (int i = 0; i < 45; ++i) {
            const double middle = 0.5 * (low + high);
            const auto candidate = integrate(previous, applied, wind, state.mass,
                                              state.dragArea, middle);
            if (candidate.position.z > 0.0) low = middle;
            else high = middle;
        }
        elapsed = 0.5 * (low + high);
        next = integrate(previous, applied, wind, state.mass, state.dragArea, elapsed);
        next.position.z = 0.0;
        state.impact = next.position;
        state.impacted = true;
    }
    state.position = next.position;
    state.velocity = next.velocity;
    state.flightTime += elapsed;
}

Vec3 predictPayloadDrift(double height, Vec3 initialVelocity, Vec3 wind,
                        double mass, double dragArea, double dt) {
    nonnegative(height, "release height");
    positive(dt, "release-prediction timestep");
    positive(mass, "payload mass");
    nonnegative(dragArea, "payload drag area");
    if (!finite(initialVelocity) || !finite(wind)) {
        throw std::invalid_argument("release velocity and wind must be finite");
    }
    PayloadState payload;
    payload.position = {0.0, 0.0, height};
    payload.velocity = initialVelocity;
    payload.mass = mass;
    payload.dragArea = dragArea;
    // Finite bound rejects upward suspension or an unsuitable timestep instead
    // of allowing a pathological predictor to hang the fleet simulation.
    const double limit = std::max(120.0, 20.0 * std::sqrt(2.0 * height / gravity));
    if (!std::isfinite(limit) || limit / dt > 1000000.0) {
        throw std::invalid_argument("release-prediction timestep requires too many steps");
    }
    for (double time = 0.0; time <= limit && !payload.impacted; time += dt) {
        advancePayload(payload, wind, dt);
    }
    if (!payload.impacted) throw std::runtime_error("payload predictor did not reach ground");
    return payload.impact;
}

double noiseDb(double power, double distance, const Config& config) {
    nonnegative(power, "acoustic source power");
    nonnegative(distance, "observer distance");
    if (!std::isfinite(config.sourceNoiseDb)) {
        throw std::invalid_argument("source noise must be finite");
    }
    if (power == 0.0) return -std::numeric_limits<double>::infinity();
    const double range = std::max(1.0, distance);
    // Source level refers to 250 electrical W at one metre. Spherical spreading
    // and a fixed 0.002 dB/m absorption approximate propagation; no reflections,
    // terrain shielding, tonal spectrum, or calibrated frequency weighting.
    return config.sourceNoiseDb + 10.0 * std::log10(power / 250.0)
        - 20.0 * std::log10(range) - 0.002 * (range - 1.0);
}
} // namespace sim
