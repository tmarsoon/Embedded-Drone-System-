#include "Simulation.h"

#include <cmath>
#include <iomanip>
#include <locale>
#include <ostream>
#include <sstream>

namespace sim {
namespace {
void number(std::ostream& output, double value) {
    if (std::isfinite(value)) output << value;
    else output << "null";
}

void quoted(std::ostream& output, const std::string& value) {
    constexpr char hex[] = "0123456789abcdef";
    output << '"';
    for (const unsigned char character : value) {
        switch (character) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (character < 0x20) {
                output << "\\u00" << hex[character >> 4] << hex[character & 0x0f];
            } else {
                output << static_cast<char>(character);
            }
        }
    }
    output << '"';
}

void vector(std::ostream& output, Vec3 value) {
    output << '[';
    number(output, value.x); output << ',';
    number(output, value.y); output << ',';
    number(output, value.z); output << ']';
}

void scalar(std::ostream& output, const char* key, double value) {
    output << ','; quoted(output, key); output << ':'; number(output, value);
}

void boolean(std::ostream& output, bool value) { output << (value ? "true" : "false"); }

void configuration(std::ostream& output, const Config& config) {
    output << "{\"drones\":" << config.drones
           << ",\"deliveries\":" << config.deliveries
           << ",\"seed\":" << config.seed;
    scalar(output, "dt_s", config.dt);
    scalar(output, "max_time_s", config.maxTime);
    scalar(output, "spoke_radius_m", config.spokeRadius);
    scalar(output, "cruise_altitude_m", config.cruiseAltitude);
    scalar(output, "altitude_spacing_m", config.altitudeSpacing);
    scalar(output, "cruise_speed_mps", config.cruiseSpeed);
    scalar(output, "vertical_speed_mps", config.verticalSpeed);
    scalar(output, "wind_speed_mps", config.windSpeed);
    scalar(output, "wind_direction_deg", config.windDirectionDegrees);
    scalar(output, "gust_sigma_mps", config.gustSigma);
    scalar(output, "gust_correlation_s", config.gustCorrelation);
    scalar(output, "wind_shear_per_m", config.windShear);
    scalar(output, "wind_forecast_bias_mps", config.windForecastBias);
    scalar(output, "max_launch_wind_mps", config.maxLaunchWind);
    scalar(output, "gps_sigma_m", config.gpsSigma);
    scalar(output, "accelerometer_sigma_mps2", config.accelerometerSigma);
    scalar(output, "gps_dropout_probability", config.gpsDropoutProbability);
    scalar(output, "vehicle_mass_kg", config.vehicleMass);
    scalar(output, "payload_mass_kg", config.payloadMass);
    scalar(output, "payload_drag_area_m2", config.payloadDragArea);
    scalar(output, "vehicle_drag_area_m2", config.vehicleDragArea);
    scalar(output, "max_thrust_n", config.maxThrust);
    scalar(output, "rotor_disk_area_m2", config.rotorDiskArea);
    scalar(output, "rotor_efficiency", config.rotorEfficiency);
    scalar(output, "battery_wh", config.batteryWh);
    scalar(output, "reserve_fraction", config.reserveFraction);
    scalar(output, "delivery_radius_m", config.deliveryRadius);
    scalar(output, "min_separation_m", config.minSeparation);
    scalar(output, "source_noise_db", config.sourceNoiseDb);
    output << ",\"noise_observer_m\":";
    vector(output, config.noiseObserver);
    output << ",\"no_fly_zones_m\":[";
    for (std::size_t index = 0; index < config.noFlyZones.size(); ++index) {
        if (index) output << ',';
        const auto& zone = config.noFlyZones[index];
        output << '[';
        number(output, zone.minX); output << ',';
        number(output, zone.minY); output << ',';
        number(output, zone.maxX); output << ',';
        number(output, zone.maxY); output << ']';
    }
    output << "]}";
}

void statistics(std::ostream& output, const MetricSummary& metric) {
    output << "{\"count\":" << metric.count;
    scalar(output, "mean", metric.mean);
    scalar(output, "stddev", metric.stddev);
    scalar(output, "p50", metric.p50);
    scalar(output, "p95", metric.p95);
    scalar(output, "ci95_low", metric.ci95Low);
    scalar(output, "ci95_high", metric.ci95High);
    output << '}';
}

void counts(std::ostream& output, int requested, int delivered, int completed,
            int withinRadius, int rejected, int emergencyReturns, int separationEvents) {
    output << "\"requested\":" << requested << ",\"delivered\":" << delivered
           << ",\"completed\":" << completed << ",\"within_radius\":" << withinRadius
           << ",\"rejected\":" << rejected << ",\"emergency_returns\":" << emergencyReturns
           << ",\"separation_events\":" << separationEvents;
}

void mission(std::ostream& output, const MissionResult& value) {
    output << "{\"job_id\":" << value.jobId << ",\"drone_id\":" << value.droneId
           << ",\"destination_m\":";
    vector(output, value.destination);
    output << ",\"impact_m\":";
    if (value.delivered) vector(output, value.impact);
    else output << "null";
    output << ",\"delivered\":"; boolean(output, value.delivered);
    output << ",\"completed\":"; boolean(output, value.completed);
    output << ",\"within_radius\":"; boolean(output, value.withinRadius);
    scalar(output, "launched_at_s", value.launchedAt);
    scalar(output, "released_at_s", value.releasedAt);
    scalar(output, "impact_at_s", value.impactAt);
    scalar(output, "completed_at_s", value.completedAt);
    output << ",\"impact_error_m\":";
    if (value.delivered) number(output, value.impactError);
    else output << "null";
    scalar(output, "energy_wh", value.energyWh);
    output << ",\"outcome\":"; quoted(output, value.outcome);
    output << '}';
}

std::ostringstream jsonStream() {
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(12);
    return output;
}

void csvQuoted(std::ostream& output, const std::string& value) {
    output << '"';
    for (char character : value) {
        if (character == '"') output << "\"\"";
        else output << character;
    }
    output << '"';
}

void csvNumber(std::ostream& output, double value) {
    if (std::isfinite(value)) output << value;
}
} // namespace

std::string resultJson(const SimulationResult& result, const Config& config) {
    auto output = jsonStream();
    output << "{\"schema_version\":1,\"mode\":\"single\",\"coordinate_frame\":\"ENU_m\",\"config\":";
    configuration(output, config);
    output << ",\"metrics\":{";
    counts(output, result.requested, result.delivered, result.completed,
           result.withinRadius, result.rejected, result.emergencyReturns, result.separationEvents);
    scalar(output, "duration_s", result.duration);
    scalar(output, "energy_wh", result.energyWh);
    scalar(output, "minimum_separation_m", result.minimumSeparation);
    scalar(output, "max_noise_db", result.maxNoiseDb);
    scalar(output, "noise_laeq_db", result.noiseLaeqDb);
    scalar(output, "noise_sel_db", result.noiseSelDb);
    scalar(output, "max_navigation_error_m", result.maxNavigationError);
    output << ",\"saturation_steps\":" << result.saturationSteps << ",\"timed_out\":";
    boolean(output, result.timedOut);
    output << "},\"missions\":[";
    for (std::size_t index = 0; index < result.missions.size(); ++index) {
        if (index) output << ',';
        mission(output, result.missions[index]);
    }
    output << "]}";
    return output.str();
}

std::string batchJson(const BatchSummary& summary, const Config& config) {
    auto output = jsonStream();
    output << "{\"schema_version\":1,\"mode\":\"monte_carlo\",\"coordinate_frame\":\"ENU_m\",\"config\":";
    configuration(output, config);
    output << ",\"metrics\":{\"runs\":" << summary.runs << ',';
    counts(output, summary.requested, summary.delivered, summary.completed,
           summary.withinRadius, summary.rejected, summary.emergencyReturns, summary.separationEvents);
    output << ",\"timed_out_runs\":" << summary.timedOutRuns;
    scalar(output, "minimum_separation_m", summary.minimumSeparation);
    output << "},\"statistics\":{\"delivery_error_m\":";
    statistics(output, summary.deliveryError);
    output << ",\"delivery_error_run_mean_m\":"; statistics(output, summary.runMeanDeliveryError);
    output << ",\"delivery_success_run_fraction\":"; statistics(output, summary.runDeliverySuccessRate);
    output << ",\"energy_wh\":"; statistics(output, summary.energyWh);
    output << ",\"duration_s\":"; statistics(output, summary.duration);
    output << ",\"max_noise_db\":"; statistics(output, summary.maxNoiseDb);
    output << ",\"noise_laeq_db\":"; statistics(output, summary.noiseLaeqDb);
    output << "},\"statistics_notes\":{\"ci_method\":\"normal approximation for the mean; not a prediction interval\","
              "\"delivery_error_ci\":\"use delivery_error_run_mean_m for independent-run inference; pooled delivery_error_m CI does not account for within-run weather dependence\","
              "\"delivery_error_conditioning\":\"errors include only impacted parcels; runs without impacts are omitted from run-mean error, while success fractions count rejected or failed requests as failures\","
              "\"success_ci\":\"normal mean interval is not clipped to [0,1]; zero observed failures do not establish zero risk\","
              "\"noise\":\"acoustic distributions exclude runs with no airborne sound exposure\"}}";
    return output.str();
}

void writeTraceCsv(std::ostream& output, const std::vector<TelemetrySample>& trace) {
    const auto previousLocale = output.getloc();
    const auto previousPrecision = output.precision();
    output.imbue(std::locale::classic());
    output << std::setprecision(12);
    output << "time_s,drone_id,phase,x_m,y_m,z_m,estimated_x_m,estimated_y_m,estimated_z_m,"
              "vx_mps,vy_mps,vz_mps,wind_x_mps,wind_y_mps,wind_z_mps,battery_wh,thrust_n,"
              "payload_attached,hub_owner\n";
    for (const auto& sample : trace) {
        csvNumber(output, sample.time); output << ',' << sample.droneId << ',';
        csvQuoted(output, sample.phase);
        const double values[] = {
            sample.position.x, sample.position.y, sample.position.z,
            sample.estimatedPosition.x, sample.estimatedPosition.y, sample.estimatedPosition.z,
            sample.velocity.x, sample.velocity.y, sample.velocity.z,
            sample.wind.x, sample.wind.y, sample.wind.z,
            sample.batteryWh, sample.thrustN
        };
        for (double value : values) { output << ','; csvNumber(output, value); }
        output << ',' << (sample.payloadAttached ? 1 : 0) << ',' << sample.hubOwner << '\n';
    }
    output.precision(previousPrecision);
    output.imbue(previousLocale);
}
} // namespace sim
