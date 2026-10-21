#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
namespace {
Vec3 limitHorizontal(Vec3 value, double limit) {
    const double speed = horizontalNorm(value);
    if (speed > limit) { value.x *= limit/speed; value.y *= limit/speed; }
    return value;
}
}

FlightController::FlightController(const Config& config) : config_(config) {}

void FlightController::reset() { integral_ = {}; }

ThrustCommand FlightController::command(Vec3 target, const Estimate& estimate,
                                        double mass, Vec3 forecastWind, double dt) {
    if (!finite(target) || !finite(estimate.position) || !finite(estimate.velocity)
        || !finite(forecastWind) || !std::isfinite(mass) || mass <= 0.0
        || !std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("controller requires finite states, positive mass and dt");
    }
    const Vec3 error = target - estimate.position;
    // Position-to-velocity cascade with a braking-distance speed envelope. It
    // models translational multirotor guidance, not attitude/motor electronics.
    Vec3 desiredVelocity{1.1*error.x, 1.1*error.y,
                         std::clamp(1.0*error.z, -config_.verticalSpeed, config_.verticalSpeed)};
    const double horizontalDistance = horizontalNorm(error);
    const double allowedSpeed = std::min(config_.cruiseSpeed,
                                         std::sqrt(2.0*2.5*horizontalDistance));
    desiredVelocity = limitHorizontal(desiredVelocity, allowedSpeed);
    const Vec3 velocityError = desiredVelocity - estimate.velocity;
    Vec3 nextIntegral = integral_ + velocityError*dt;
    nextIntegral.x = std::clamp(nextIntegral.x, -2.0, 2.0);
    nextIntegral.y = std::clamp(nextIntegral.y, -2.0, 2.0);
    nextIntegral.z = std::clamp(nextIntegral.z, -2.0, 2.0);
    Vec3 acceleration = velocityError*2.2 + nextIntegral*0.12;
    acceleration = limitHorizontal(acceleration, 3.5);
    acceleration.z = std::clamp(acceleration.z, -2.5, 2.5);

    // Compensate forecast aerodynamic drag; unmodelled gusts remain disturbances.
    const Vec3 relative = estimate.velocity - forecastWind;
    const Vec3 drag = relative*(-0.5*airDensity*config_.vehicleDragArea*norm(relative));
    Vec3 thrust = (acceleration + Vec3{0, 0, gravity})*mass - drag;
    thrust.z = std::max(0.0, thrust.z);
    const double magnitude = norm(thrust);
    const bool saturated = magnitude > config_.maxThrust;
    if (saturated) {
        thrust = thrust*(config_.maxThrust/magnitude);
    } else {
        integral_ = nextIntegral; // Freeze integration while actuator-limited.
    }
    return {thrust, saturated};
}
} // namespace sim
