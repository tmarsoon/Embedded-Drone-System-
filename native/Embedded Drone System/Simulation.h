#pragma once

#include "PathPlanning.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iosfwd>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace sim {
constexpr double gravity = 9.80665;
constexpr double airDensity = 1.225;
constexpr double pi = 3.14159265358979323846;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator+(Vec3 b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vec3& operator+=(Vec3 b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
};
double norm(Vec3 v);
double horizontalNorm(Vec3 v);
bool finite(Vec3 v);
struct NoFlyZone { double minX, minY, maxX, maxY; };

struct Config {
    int drones = 3, deliveries = 6;
    std::uint32_t seed = 42;
    double dt = 0.1, maxTime = 1200.0;
    double spokeRadius = 150.0, cruiseAltitude = 40.0, altitudeSpacing = 12.0;
    double cruiseSpeed = 8.0, verticalSpeed = 3.0;
    double windSpeed = 3.0, windDirectionDegrees = 30.0;
    double gustSigma = 0.8, gustCorrelation = 3.0, windShear = 0.002;
    double windForecastBias = 0.0, maxLaunchWind = 12.0;
    double gpsSigma = 0.25, accelerometerSigma = 0.04;
    double gpsDropoutProbability = 0.01;
    double vehicleMass = 2.0, payloadMass = 0.3;
    double payloadDragArea = 0.018, vehicleDragArea = 0.07;
    double maxThrust = 48.0, rotorDiskArea = 0.28, rotorEfficiency = 0.65;
    double batteryWh = 160.0, reserveFraction = 0.2;
    double deliveryRadius = 5.0, minSeparation = 8.0;
    double sourceNoiseDb = 78.0;
    Vec3 noiseObserver{40.0, 20.0, 1.5};
    std::vector<NoFlyZone> noFlyZones;
};
void validateConfig(const Config& config);

class WindField {
public:
    explicit WindField(const Config& config);
    void step(double dt);
    Vec3 at(Vec3 position) const;
    Vec3 forecastAt(Vec3 position) const;
private:
    Config config_;
    Vec3 gust_;
    std::mt19937 random_;
    std::normal_distribution<double> normal_{0.0, 1.0};
};

struct VehicleState {
    Vec3 position, velocity, acceleration;
    double batteryWh = 0.0, energyWh = 0.0;
};
double electricalPower(double thrust, double airspeed, const Config& config);
double advanceVehicle(VehicleState& state, Vec3 thrust, Vec3 wind,
                      double mass, double dt, const Config& config);

struct PayloadState {
    Vec3 position, velocity, impact;
    double mass = 0.3, dragArea = 0.018, flightTime = 0.0;
    bool impacted = false;
};
void advancePayload(PayloadState& state, Vec3 wind, double dt);
Vec3 predictPayloadDrift(double height, Vec3 initialVelocity, Vec3 wind,
                        double mass, double dragArea, double dt = 0.02);
double noiseDb(double power, double distance, const Config& config);

struct Estimate { Vec3 position, velocity; bool gpsAvailable = true; };
class NavigationFilter {
public:
    NavigationFilter(Vec3 initial, const Config& config, std::uint32_t seed);
    Estimate update(const VehicleState& truth, double dt);
    Estimate state() const;
private:
    struct Axis { double position=0, velocity=0, p00=1, p01=0, p10=0, p11=1; };
    Config config_;
    std::array<Axis, 3> axes_;
    double gpsTimer_ = 0.0;
    bool gpsAvailable_ = true;
    std::mt19937 random_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

struct ThrustCommand { Vec3 force; bool saturated = false; };
class FlightController {
public:
    explicit FlightController(const Config& config);
    ThrustCommand command(Vec3 target, const Estimate& estimate, double mass,
                          Vec3 forecastWind, double dt);
    void reset();
private:
    Config config_;
    Vec3 integral_;
};

struct EnergyAssessment {
    bool sufficient = false;
    double requiredWh = 0.0, projectedRemainingWh = 0.0;
};
EnergyAssessment assessMissionEnergy(double batteryWh, double horizontalDistance,
                                     double altitude, double mass,
                                     const Config& config);

class HubScheduler {
public:
    explicit HubScheduler(int jobs);
    std::optional<int> nextJob();
    bool acquire(int droneId);
    void release(int droneId);
    std::optional<int> owner() const;
    bool empty() const;
private:
    std::deque<int> pending_;
    std::optional<int> owner_;
};

struct MissionResult {
    int jobId = 0, droneId = 0;
    Vec3 destination, impact;
    bool delivered = false, completed = false, withinRadius = false;
    double launchedAt = 0.0, releasedAt = 0.0, impactAt = 0.0, completedAt = 0.0;
    double impactError = 0.0, energyWh = 0.0;
    std::string outcome;
};
struct TelemetrySample {
    double time = 0.0;
    int droneId = 0;
    std::string phase;
    Vec3 position, estimatedPosition, velocity, wind;
    double batteryWh = 0.0, thrustN = 0.0;
    bool payloadAttached = false;
    int hubOwner = -1;
};
struct SimulationResult {
    int requested = 0, delivered = 0, completed = 0, withinRadius = 0;
    int rejected = 0, emergencyReturns = 0, separationEvents = 0, saturationSteps = 0;
    double duration = 0.0, energyWh = 0.0;
    double minimumSeparation = std::numeric_limits<double>::infinity();
    double maxNoiseDb = -std::numeric_limits<double>::infinity();
    double noiseLaeqDb = -std::numeric_limits<double>::infinity();
    double noiseSelDb = -std::numeric_limits<double>::infinity();
    double maxNavigationError = 0.0;
    bool timedOut = false;
    std::vector<MissionResult> missions;
    std::vector<TelemetrySample> trace;
};
SimulationResult runSimulation(const Config& config, bool recordTrace = false);

struct MetricSummary {
    std::size_t count = 0;
    double mean = 0.0, stddev = 0.0, p50 = 0.0, p95 = 0.0;
    double ci95Low = 0.0, ci95High = 0.0;
};
MetricSummary summarizeMetric(std::vector<double> values);
struct BatchSummary {
    int runs = 0, requested = 0, delivered = 0, completed = 0, withinRadius = 0;
    int rejected = 0, emergencyReturns = 0, separationEvents = 0, timedOutRuns = 0;
    MetricSummary deliveryError, runMeanDeliveryError, runDeliverySuccessRate;
    MetricSummary energyWh, duration, maxNoiseDb, noiseLaeqDb;
    double minimumSeparation = std::numeric_limits<double>::infinity();
};
BatchSummary summarizeRuns(const std::vector<SimulationResult>& runs);
std::string resultJson(const SimulationResult& result, const Config& config);
std::string batchJson(const BatchSummary& summary, const Config& config);
void writeTraceCsv(std::ostream& output, const std::vector<TelemetrySample>& trace);
} // namespace sim
