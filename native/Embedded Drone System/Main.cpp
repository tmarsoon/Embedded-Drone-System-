#include "Main.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
const char* help = R"(Drone hub-and-spoke research simulator (C++17)
Usage: drone_sim [options]
  --runs N                 Independent seeded scenarios (default 1)
  --seed N                 First random seed (default 42; then seed+run)
  --drones N               Fleet size (default 3)
  --deliveries N           Jobs returning to hub (default 6)
  --dt SECONDS             Integration timestep (default 0.1)
  --duration SECONDS       Scenario time limit (default 1200)
  --radius METRES          Spoke distance (default 150)
  --altitude METRES        Lowest cruise lane (default 40)
  --lane-spacing METRES    Vertical separation of cruise lanes (default 12)
  --speed MPS              Cruise speed limit (default 8)
  --vertical-speed MPS     Climb/descent speed limit (default 3)
  --wind MPS               Mean surface wind (default 3)
  --wind-direction DEG     Wind vector direction from +x (default 30)
  --gust MPS               Stationary gust standard deviation (default 0.8)
  --gust-correlation SEC   Ornstein-Uhlenbeck correlation time (default 3)
  --wind-shear PER_METRE   Linear altitude shear (default 0.002)
  --wind-bias MPS          Forecast speed error (default 0)
  --launch-wind-limit MPS  Preflight mean-wind limit (default 12)
  --gps-noise METRES       GPS noise standard deviation (default 0.25)
  --accel-noise MPS2       Accelerometer noise standard deviation (default 0.04)
  --gps-dropout FRACTION   Independent GPS update dropout probability (default .01)
  --mass KG                Empty aircraft mass (default 2)
  --payload KG             Parcel mass (default .3)
  --payload-drag-area M2   Parcel Cd times frontal area (default .018)
  --vehicle-drag-area M2   Aircraft Cd times frontal area (default .07)
  --max-thrust NEWTONS     Aircraft thrust limit (default 48)
  --battery WH             Per-aircraft battery capacity (default 160)
  --reserve FRACTION       Battery reserve fraction (default .2)
  --delivery-radius METRES Parcel acceptance radius (default 5)
  --noise-source DB        A-weighted level at 1m, reference power 250W (default 78)
  --observer X Y Z         Acoustic observer position in metres
  --no-fly XMIN YMIN XMAX YMAX   Add an axis-aligned no-fly column
  --output DIRECTORY       Write summary.json and runs.jsonl
  --trace                  Also write first-run telemetry.csv and first_run.json
                           Requires --output. No real-time sleeping or hardware.
  --help                   Show this help

stdout is machine-readable JSON. Outcomes such as weather rejection and missed
deliveries are reported as data; invalid input or execution errors exit nonzero.
)";

double number(const std::string& value, const std::string& name) {
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(result)) {
        throw std::invalid_argument(name + " requires a finite number");
    }
    return result;
}

long long integer(const std::string& value, const std::string& name) {
    std::size_t consumed = 0;
    const long long result = std::stoll(value, &consumed);
    if (consumed != value.size()) throw std::invalid_argument(name + " requires an integer");
    return result;
}

void save(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot open output: " + path.string());
    output << contents << '\n';
    if (!output) throw std::runtime_error("cannot write output: " + path.string());
}
} // namespace

int main(int argc, char** argv) {
    try {
        sim::Config config;
        int runs = 1;
        bool trace = false;
        std::filesystem::path outputDirectory;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            const auto next = [&]() -> std::string {
                if (++i >= argc) throw std::invalid_argument("missing value for " + option);
                return argv[i];
            };
            const auto value = [&]() { return number(next(), option); };
            if (option == "--help") { std::cout << help; return 0; }
            if (option == "--trace") { trace = true; continue; }
            if (option == "--output") { outputDirectory = next(); continue; }
            if (option == "--seed") {
                const auto n = integer(next(), option);
                if (n < 0 || n > std::numeric_limits<std::uint32_t>::max())
                    throw std::invalid_argument("seed must be a uint32 integer");
                config.seed = static_cast<std::uint32_t>(n);
                continue;
            }
            if (option == "--runs" || option == "--drones" || option == "--deliveries") {
                const auto n = integer(next(), option);
                if (n < 1 || n > 10000) throw std::invalid_argument(option + " must be in [1,10000]");
                if (option == "--runs") runs = static_cast<int>(n);
                if (option == "--drones") config.drones = static_cast<int>(n);
                if (option == "--deliveries") config.deliveries = static_cast<int>(n);
                continue;
            }
            if (option == "--observer") {
                const double x=value(), y=value(), z=value();
                config.noiseObserver = {x,y,z};
                continue;
            }
            if (option == "--no-fly") {
                const double x1=value(), y1=value(), x2=value(), y2=value();
                config.noFlyZones.push_back({x1,y1,x2,y2});
                continue;
            }
            if (option == "--dt") config.dt = value();
            else if (option == "--duration") config.maxTime = value();
            else if (option == "--radius") config.spokeRadius = value();
            else if (option == "--altitude") config.cruiseAltitude = value();
            else if (option == "--lane-spacing") config.altitudeSpacing = value();
            else if (option == "--speed") config.cruiseSpeed = value();
            else if (option == "--vertical-speed") config.verticalSpeed = value();
            else if (option == "--wind") config.windSpeed = value();
            else if (option == "--wind-direction") config.windDirectionDegrees = value();
            else if (option == "--gust") config.gustSigma = value();
            else if (option == "--gust-correlation") config.gustCorrelation = value();
            else if (option == "--wind-shear") config.windShear = value();
            else if (option == "--wind-bias") config.windForecastBias = value();
            else if (option == "--launch-wind-limit") config.maxLaunchWind = value();
            else if (option == "--gps-noise") config.gpsSigma = value();
            else if (option == "--accel-noise") config.accelerometerSigma = value();
            else if (option == "--gps-dropout") config.gpsDropoutProbability = value();
            else if (option == "--mass") config.vehicleMass = value();
            else if (option == "--payload") config.payloadMass = value();
            else if (option == "--payload-drag-area") config.payloadDragArea = value();
            else if (option == "--vehicle-drag-area") config.vehicleDragArea = value();
            else if (option == "--max-thrust") config.maxThrust = value();
            else if (option == "--battery") config.batteryWh = value();
            else if (option == "--reserve") config.reserveFraction = value();
            else if (option == "--delivery-radius") config.deliveryRadius = value();
            else if (option == "--noise-source") config.sourceNoiseDb = value();
            else throw std::invalid_argument("unknown option: " + option);
        }
        if (trace && outputDirectory.empty()) throw std::invalid_argument("--trace requires --output");
        sim::validateConfig(config);
        std::vector<sim::SimulationResult> results;
        results.reserve(static_cast<std::size_t>(runs));
        for (int run = 0; run < runs; ++run) {
            sim::Config trial = config;
            trial.seed += static_cast<std::uint32_t>(run);
            results.push_back(sim::runSimulation(trial, trace && run == 0));
        }
        const std::string summary = runs == 1 ? sim::resultJson(results.front(), config)
            : sim::batchJson(sim::summarizeRuns(results), config);
        if (!outputDirectory.empty()) {
            std::filesystem::create_directories(outputDirectory);
            save(outputDirectory / "summary.json", summary);
            std::ofstream individual(outputDirectory / "runs.jsonl");
            if (!individual) throw std::runtime_error("cannot open runs.jsonl");
            for (int run = 0; run < runs; ++run) {
                auto trial = config;
                trial.seed += static_cast<std::uint32_t>(run);
                individual << sim::resultJson(results[run], trial) << '\n';
            }
            if (!individual) throw std::runtime_error("cannot write runs.jsonl");
            if (trace) {
                std::ofstream telemetry(outputDirectory / "telemetry.csv");
                if (!telemetry) throw std::runtime_error("cannot open telemetry.csv");
                sim::writeTraceCsv(telemetry, results.front().trace);
                if (!telemetry) throw std::runtime_error("cannot write telemetry.csv");
                save(outputDirectory / "first_run.json", sim::resultJson(results.front(), config));
            }
        }
        std::cout << summary << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "drone_sim: " << error.what() << '\n';
        return 1;
    }
}
