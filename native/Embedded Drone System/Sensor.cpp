#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

NavigationFilter::NavigationFilter(Vec3 initial, const Config& config,
                                   std::uint32_t seed)
    : config_(config), random_(seed ^ 0xbb67ae85U) {
    if (!finite(initial) || !std::isfinite(config_.gpsSigma)
        || config_.gpsSigma < 0.0 || !std::isfinite(config_.accelerometerSigma)
        || config_.accelerometerSigma < 0.0
        || !std::isfinite(config_.gpsDropoutProbability)
        || config_.gpsDropoutProbability < 0.0 || config_.gpsDropoutProbability > 1.0) {
        throw std::invalid_argument("invalid navigation sensor configuration");
    }
    axes_[0].position = initial.x;
    axes_[1].position = initial.y;
    axes_[2].position = initial.z;
}

Estimate NavigationFilter::update(const VehicleState& truth, double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || !finite(truth.position)
        || !finite(truth.acceleration)) {
        throw std::invalid_argument("navigation input and positive timestep must be finite");
    }
    const double positions[] = {truth.position.x, truth.position.y, truth.position.z};
    const double accelerations[] = {truth.acceleration.x, truth.acceleration.y,
                                    truth.acceleration.z};
    // Independent 2x2 position/velocity Kalman blocks constitute the full six-
    // state filter. Sensor acceleration drives prediction; truth velocity is
    // never observed. Small model variance handles unmodelled acceleration.
    const double accelerationVariance = config_.accelerometerSigma
        * config_.accelerometerSigma + 0.01;
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
    gpsTimer_ += dt;
    const bool sampleGps = gpsTimer_ >= 0.2 - 1e-12;
    if (sampleGps) {
        const double intervals = std::floor((gpsTimer_ + 1e-12) / 0.2);
        gpsTimer_ = std::max(0.0, gpsTimer_ - intervals * 0.2);
    }
    gpsAvailable_ = sampleGps && uniform_(random_) >= config_.gpsDropoutProbability;
    const double measurementVariance = std::max(1e-10, config_.gpsSigma * config_.gpsSigma);
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        Axis& axis = axes_[index];
        const double acceleration = accelerations[index]
            + config_.accelerometerSigma * normal_(random_);
        axis.position += axis.velocity * dt + 0.5 * acceleration * dt2;
        axis.velocity += acceleration * dt;
        const double p00 = axis.p00 + dt * (axis.p01 + axis.p10)
            + dt2 * axis.p11 + 0.25 * accelerationVariance * dt4;
        const double p01 = axis.p01 + dt * axis.p11
            + 0.5 * accelerationVariance * dt3;
        const double p10 = axis.p10 + dt * axis.p11
            + 0.5 * accelerationVariance * dt3;
        const double p11 = axis.p11 + accelerationVariance * dt2;
        axis.p00 = p00; axis.p01 = p01; axis.p10 = p10; axis.p11 = p11;
        if (gpsAvailable_) {
            const double measurement = positions[index]
                + config_.gpsSigma * normal_(random_);
            const double innovation = measurement - axis.position;
            const double denominator = p00 + measurementVariance;
            const double k0 = p00 / denominator, k1 = p10 / denominator;
            axis.position += k0 * innovation;
            axis.velocity += k1 * innovation;
            // Joseph form: (I-KH)P(I-KH)' + KRK'. This retains symmetry and
            // nonnegative diagonal variances even for nearly noiseless GPS.
            const double oneMinusK0 = 1.0 - k0;
            axis.p00 = oneMinusK0 * oneMinusK0 * p00
                + k0 * k0 * measurementVariance;
            axis.p01 = oneMinusK0 * (p01 - k1 * p00)
                + k0 * k1 * measurementVariance;
            axis.p10 = oneMinusK0 * (p10 - k1 * p00)
                + k0 * k1 * measurementVariance;
            axis.p11 = p11 - k1 * (p01 + p10) + k1 * k1 * p00
                + k1 * k1 * measurementVariance;
            const double symmetric = 0.5 * (axis.p01 + axis.p10);
            axis.p01 = symmetric;
            axis.p10 = symmetric;
            axis.p00 = std::max(axis.p00, 0.0);
            axis.p11 = std::max(axis.p11, 0.0);
        }
        if (!std::isfinite(axis.position) || !std::isfinite(axis.velocity)
            || !std::isfinite(axis.p00) || !std::isfinite(axis.p11)) {
            throw std::runtime_error("navigation filter became non-finite");
        }
    }
    return state();
}

Estimate NavigationFilter::state() const {
    return {{axes_[0].position, axes_[1].position, axes_[2].position},
            {axes_[0].velocity, axes_[1].velocity, axes_[2].velocity}, gpsAvailable_};
}
} // namespace sim
