#include "Simulation.h"

#include <cmath>
#include <stdexcept>

namespace sim {
EnergyAssessment assessMissionEnergy(double batteryWh, double horizontalDistance,
                                     double altitude, double mass,
                                     const Config& config) {
    if (!std::isfinite(batteryWh) || batteryWh < 0.0 ||
        !std::isfinite(horizontalDistance) || horizontalDistance < 0.0 ||
        !std::isfinite(altitude) || altitude < 0.0 ||
        !std::isfinite(mass) || mass <= 0.0 ||
        !std::isfinite(config.cruiseSpeed) || config.cruiseSpeed <= 0.0 ||
        !std::isfinite(config.verticalSpeed) || config.verticalSpeed <= 0.0 ||
        !std::isfinite(config.batteryWh) || config.batteryWh <= 0.0 ||
        !std::isfinite(config.reserveFraction) || config.reserveFraction < 0.0 ||
        config.reserveFraction >= 1.0) {
        throw std::invalid_argument("invalid energy assessment input");
    }
    // This is a transparent conservative energy forecast, not a learned
    // maintenance/failure predictor. horizontalDistance is the TOTAL remaining
    // horizontal travel, so launch callers should supply an out-and-back route.
    const double cruiseTime = horizontalDistance / config.cruiseSpeed;
    const double verticalTime = 2.0 * altitude / config.verticalSpeed;
    constexpr double serviceHoverSeconds = 15.0;
    constexpr double uncertaintyMargin = 1.3;
    const double hoverPower = electricalPower(mass * gravity, 0.0, config);
    const double cruisePower = electricalPower(mass * gravity, config.cruiseSpeed, config);
    const double required = uncertaintyMargin *
        (cruisePower * cruiseTime + hoverPower * (verticalTime + serviceHoverSeconds)) / 3600.0;
    const double reserve = config.reserveFraction * config.batteryWh;
    return {batteryWh >= required + reserve, required, batteryWh - required};
}
} // namespace sim
