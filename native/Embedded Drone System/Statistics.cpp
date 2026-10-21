#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
MetricSummary summarizeMetric(std::vector<double> values) {
    MetricSummary summary;
    if (values.empty()) return summary;
    for (double value : values) {
        if (!std::isfinite(value)) throw std::invalid_argument("statistics require finite samples");
    }
    summary.count = values.size();
    // Welford avoids cancellation when all samples have a large common offset.
    double sumSquares = 0.0;
    std::size_t count = 0;
    for (double value : values) {
        ++count;
        const double delta = value - summary.mean;
        summary.mean += delta / static_cast<double>(count);
        sumSquares += delta * (value - summary.mean);
    }
    if (count > 1) summary.stddev = std::sqrt(std::max(0.0, sumSquares / static_cast<double>(count - 1)));
    std::sort(values.begin(), values.end());
    const auto percentile = [&values](double probability) {
        const double index = probability * static_cast<double>(values.size() - 1);
        const auto lower = static_cast<std::size_t>(std::floor(index));
        const auto upper = static_cast<std::size_t>(std::ceil(index));
        return values[lower] + (index - static_cast<double>(lower)) * (values[upper] - values[lower]);
    };
    summary.p50 = percentile(0.5);
    summary.p95 = percentile(0.95);
    // A normal approximation for the MEAN, not an individual-outcome interval.
    // For pooled delivery errors, parcels in one run share weather and are not
    // independent: use run-level repetitions for inference on a real fleet.
    const double halfWidth = 1.959963984540054 * summary.stddev / std::sqrt(static_cast<double>(count));
    summary.ci95Low = summary.mean - halfWidth;
    summary.ci95High = summary.mean + halfWidth;
    return summary;
}

BatchSummary summarizeRuns(const std::vector<SimulationResult>& runs) {
    BatchSummary summary;
    summary.runs = static_cast<int>(runs.size());
    std::vector<double> errors, runMeanErrors, runSuccessRates;
    std::vector<double> energy, duration, maximumNoise, equivalentNoise;
    for (const auto& run : runs) {
        summary.requested += run.requested;
        summary.delivered += run.delivered;
        summary.completed += run.completed;
        summary.withinRadius += run.withinRadius;
        summary.rejected += run.rejected;
        summary.emergencyReturns += run.emergencyReturns;
        summary.separationEvents += run.separationEvents;
        summary.timedOutRuns += run.timedOut ? 1 : 0;
        if (std::isfinite(run.minimumSeparation)) {
            summary.minimumSeparation = std::min(summary.minimumSeparation, run.minimumSeparation);
        }
        energy.push_back(run.energyWh);
        duration.push_back(run.duration);
        if (std::isfinite(run.maxNoiseDb)) maximumNoise.push_back(run.maxNoiseDb);
        if (std::isfinite(run.noiseLaeqDb)) equivalentNoise.push_back(run.noiseLaeqDb);
        std::vector<double> currentErrors;
        for (const auto& mission : run.missions) {
            if (mission.delivered) {
                errors.push_back(mission.impactError);
                currentErrors.push_back(mission.impactError);
            }
        }
        if (!currentErrors.empty()) runMeanErrors.push_back(summarizeMetric(std::move(currentErrors)).mean);
        if (run.requested > 0) {
            runSuccessRates.push_back(static_cast<double>(run.withinRadius) / run.requested);
        }
    }
    summary.deliveryError = summarizeMetric(std::move(errors));
    summary.runMeanDeliveryError = summarizeMetric(std::move(runMeanErrors));
    summary.runDeliverySuccessRate = summarizeMetric(std::move(runSuccessRates));
    summary.energyWh = summarizeMetric(std::move(energy));
    summary.duration = summarizeMetric(std::move(duration));
    summary.maxNoiseDb = summarizeMetric(std::move(maximumNoise));
    summary.noiseLaeqDb = summarizeMetric(std::move(equivalentNoise));
    return summary;
}
} // namespace sim
