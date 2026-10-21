#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {
enum class Phase { Parked, Ascending, Outbound, Releasing, Returning, Holding, Landing };
const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::Parked: return "parked";
        case Phase::Ascending: return "ascending";
        case Phase::Outbound: return "outbound";
        case Phase::Releasing: return "releasing";
        case Phase::Returning: return "returning";
        case Phase::Holding: return "holding";
        case Phase::Landing: return "landing";
    }
    return "unknown";
}

struct Aircraft {
    int id;
    Vec3 pad;
    double altitude;
    VehicleState state;
    NavigationFilter filter;
    FlightController controller;
    Phase phase = Phase::Parked;
    int mission = -1;
    bool payloadAttached = false, emergency = false, disabled = false, offHubLanding = false;
    double energyAtLaunch = 0.0;
    double holdingSince = 0.0;
    Vec3 releasePoint;
    std::vector<Vec3> route;
    std::vector<Vec3> plannedReturnRoute;
    std::size_t waypoint = 0;
    Aircraft(int number, const Config& config)
        : id(number), pad{20.0*(number - (config.drones-1)/2.0), 0, 0},
          altitude(config.cruiseAltitude + config.altitudeSpacing*number),
          filter(pad, config, config.seed + 104729u*(static_cast<unsigned>(number)+1u)),
          controller(config) {
        state.position = pad;
        state.batteryWh = config.batteryWh;
    }
    Vec3 holdingPoint() const { return {pad.x, pad.y-50.0, altitude}; }
};
struct FallingPayload { PayloadState state; int mission; double releaseTime; };

bool inside(Vec3 point, const NoFlyZone& zone, double margin = 0.0) {
    return point.x >= zone.minX-margin && point.x <= zone.maxX+margin
        && point.y >= zone.minY-margin && point.y <= zone.maxY+margin;
}

std::vector<Vec3> planRoute(Vec3 start, Vec3 target, const Config& config) {
    if (config.noFlyZones.empty()) return {target};
    constexpr double baseResolution = 10.0;
    constexpr double clearance = 8.0;
    double minX = std::min(start.x, target.x)-80.0, maxX = std::max(start.x, target.x)+80.0;
    double minY = std::min(start.y, target.y)-80.0, maxY = std::max(start.y, target.y)+80.0;
    for (const auto& zone : config.noFlyZones) {
        if (inside(start, zone, clearance) || inside(target, zone, clearance)) return {};
        // Include obstacles intersecting the initial route bounding area. Faraway
        // obstacles do not inflate grid allocations.
        if (zone.maxX >= minX && zone.minX <= maxX && zone.maxY >= minY && zone.minY <= maxY) {
            minX = std::min(minX, zone.minX-30.0); maxX = std::max(maxX, zone.maxX+30.0);
            minY = std::min(minY, zone.minY-30.0); maxY = std::max(maxY, zone.maxY+30.0);
        }
    }
    const double resolution = std::max(baseResolution, std::max(maxX-minX, maxY-minY)/200.0);
    minX = std::floor(minX/resolution)*resolution;
    minY = std::floor(minY/resolution)*resolution;
    const int width = static_cast<int>(std::ceil((maxX-minX)/resolution))+1;
    const int height = static_cast<int>(std::ceil((maxY-minY)/resolution))+1;
    std::vector<std::vector<int>> grid(width, std::vector<int>(height, 0));
    for (int x=0; x<width; ++x) for (int y=0; y<height; ++y) {
        const Vec3 point{minX+x*resolution, minY+y*resolution, target.z};
        for (const auto& zone : config.noFlyZones) {
            if (inside(point, zone, clearance+resolution*0.5)) { grid[x][y]=1; break; }
        }
    }
    auto coordinates = [&](Vec3 point) {
        return std::pair<int,int>{static_cast<int>(std::lround((point.x-minX)/resolution)),
                                  static_cast<int>(std::lround((point.y-minY)/resolution))};
    };
    const auto from = coordinates(start), to = coordinates(target);
    AStarPlanner planner(width, height, grid);
    const auto reversed = planner.plan(from.first, from.second, to.first, to.second);
    if (reversed.empty()) return {};
    std::vector<Vec3> route;
    for (auto it=reversed.rbegin(); it!=reversed.rend(); ++it) {
        const Vec3 point{minX+it->x*resolution, minY+it->y*resolution, target.z};
        if (norm(point-start) > 1.0) route.push_back(point);
    }
    // Keep corner vertices, removing intermediate cells along straight legs.
    std::vector<Vec3> simplified;
    for (const Vec3 point : route) {
        while (simplified.size() >= 2) {
            const Vec3 a=simplified[simplified.size()-2], b=simplified.back();
            const Vec3 ab=b-a, bc=point-b;
            if (std::abs(ab.x*bc.y-ab.y*bc.x) > 1e-8) break;
            simplified.pop_back();
        }
        simplified.push_back(point);
    }
    if (simplified.empty() || norm(simplified.back()-target)>1e-8) simplified.push_back(target);
    return simplified;
}

bool nearTarget(const Aircraft& aircraft, Vec3 target, double positionTolerance=0.8,
                double speedTolerance=1.0) {
    const Estimate estimate = aircraft.filter.state();
    return norm(estimate.position-target) < positionTolerance
        && norm(estimate.velocity) < speedTolerance;
}

bool setRoute(Aircraft& aircraft, Vec3 target, const Config& config) {
    aircraft.route = planRoute(aircraft.filter.state().position, target, config);
    aircraft.waypoint = 0;
    aircraft.controller.reset();
    return !aircraft.route.empty();
}

void emergencyLanding(Aircraft& aircraft, MissionResult& mission,
                      SimulationResult& result, const char* reason) {
    if (!aircraft.emergency) ++result.emergencyReturns;
    aircraft.emergency=true;
    aircraft.disabled=true;
    aircraft.offHubLanding=true;
    aircraft.phase=Phase::Landing;
    const Vec3 current=aircraft.filter.state().position;
    aircraft.route={{current.x,current.y,0.0}};
    aircraft.waypoint=0;
    aircraft.controller.reset();
    mission.outcome=reason;
}

Vec3 routeTarget(Aircraft& aircraft) {
    if (aircraft.route.empty()) return aircraft.filter.state().position;
    if (aircraft.waypoint+1 < aircraft.route.size()
        && nearTarget(aircraft, aircraft.route[aircraft.waypoint], 1.0, 1.4)) {
        ++aircraft.waypoint;
    }
    return aircraft.route[aircraft.waypoint];
}

bool routeDone(const Aircraft& aircraft) {
    return !aircraft.route.empty() && aircraft.waypoint+1 == aircraft.route.size()
        && nearTarget(aircraft, aircraft.route.back());
}

double routeDistance(Vec3 start, const std::vector<Vec3>& route) {
    double distance = 0.0;
    for (const Vec3 waypoint : route) {
        distance += horizontalNorm(waypoint-start);
        start = waypoint;
    }
    return distance;
}

bool segmentEntersColumn(Vec3 from, Vec3 to, Vec3 pad,
                         double radius, double top, double margin) {
    // Restrict to the segment portion overlapping the ascent/descent altitude
    // range, then minimize horizontal distance over that interval.
    const Vec3 delta=to-from;
    double lower=0.0, upper=1.0;
    if (std::abs(delta.z)<1e-12) {
        if (from.z < -margin || from.z > top) return false;
    } else {
        const double a=(-margin-from.z)/delta.z, b=(top-from.z)/delta.z;
        lower=std::max(0.0,std::min(a,b));
        upper=std::min(1.0,std::max(a,b));
        if (lower>upper) return false;
    }
    const double horizontalSquared=delta.x*delta.x+delta.y*delta.y;
    const double projection=(from.x-pad.x)*delta.x+(from.y-pad.y)*delta.y;
    const double fraction=horizontalSquared>0.0
        ? std::clamp(-projection/horizontalSquared,lower,upper):lower;
    return horizontalNorm(from+delta*fraction-pad)<=radius;
}

bool corridorClear(const Aircraft& candidate, const std::vector<Aircraft>& fleet,
                   const Config& config) {
    const double margin=std::max(2.0,3.0*config.gpsSigma);
    const double radius=config.minSeparation+margin;
    const double top=candidate.altitude+margin;
    for (const auto& other:fleet) {
        if (other.id==candidate.id || other.phase==Phase::Parked) continue;
        Vec3 previous=other.filter.state().position;
        if (segmentEntersColumn(previous,previous,candidate.pad,radius,top,margin)) return false;
        auto clearSegment = [&](Vec3 waypoint) {
            const bool clear=!segmentEntersColumn(previous,waypoint,candidate.pad,radius,top,margin);
            previous=waypoint;
            return clear;
        };
        // Holding traffic is stationary until it acquires the shared token.
        // Reserving its speculative landing route here could cause deadlock.
        if (other.phase==Phase::Holding) continue;
        for (std::size_t waypoint=other.waypoint; waypoint<other.route.size(); ++waypoint)
            if (!clearSegment(other.route[waypoint])) return false;
        // Delivery aircraft can turn inbound during the candidate's climb or
        // descent. Include their known return-to-hold path before admitting the
        // candidate, but never reserve an already-completed segment.
        if (other.phase==Phase::Outbound || other.phase==Phase::Releasing) {
            for (const Vec3 waypoint:other.plannedReturnRoute)
                if (!clearSegment(waypoint)) return false;
        }
    }
    return true;
}

void positive(double value, const char* name, double maximum=1e6) {
    if (!std::isfinite(value) || value<=0.0 || value>maximum)
        throw std::invalid_argument(std::string(name)+" must be positive, finite and in range");
}
void nonnegative(double value, const char* name, double maximum=1e6) {
    if (!std::isfinite(value) || value<0.0 || value>maximum)
        throw std::invalid_argument(std::string(name)+" must be nonnegative, finite and in range");
}
}

void validateConfig(const Config& c) {
    if (c.drones < 1 || c.drones > 32 || c.deliveries < 1 || c.deliveries > 1000)
        throw std::invalid_argument("drones must be 1..32 and deliveries 1..1000");
    positive(c.dt,"dt",0.25);
    if (c.dt < 0.01) throw std::invalid_argument("dt must be at least 0.01 seconds");
    positive(c.maxTime,"duration",7200.0);
    positive(c.spokeRadius,"spoke radius",2000.0);
    positive(c.cruiseAltitude,"cruise altitude",300.0);
    nonnegative(c.altitudeSpacing,"altitude spacing",100.0);
    positive(c.cruiseSpeed,"cruise speed",25.0);
    positive(c.verticalSpeed,"vertical speed",8.0);
    nonnegative(c.windSpeed,"wind speed",60.0);
    if (!std::isfinite(c.windDirectionDegrees)) throw std::invalid_argument("wind direction must be finite");
    nonnegative(c.gustSigma,"gust standard deviation",20.0);
    positive(c.gustCorrelation,"gust correlation",1000.0);
    nonnegative(c.windShear,"wind shear",0.1);
    if (!std::isfinite(c.windForecastBias) || std::abs(c.windForecastBias)>60.0)
        throw std::invalid_argument("forecast bias must be finite with magnitude at most 60 m/s");
    positive(c.maxLaunchWind,"launch wind limit",60.0);
    nonnegative(c.gpsSigma,"GPS noise",100.0);
    nonnegative(c.accelerometerSigma,"accelerometer noise",10.0);
    if (!std::isfinite(c.gpsDropoutProbability) || c.gpsDropoutProbability<0 || c.gpsDropoutProbability>1)
        throw std::invalid_argument("GPS dropout probability must be in [0,1]");
    positive(c.vehicleMass,"vehicle mass",100.0);
    positive(c.payloadMass,"payload mass",20.0);
    nonnegative(c.payloadDragArea,"payload drag area",10.0);
    nonnegative(c.vehicleDragArea,"vehicle drag area",10.0);
    positive(c.maxThrust,"maximum thrust",5000.0);
    positive(c.rotorDiskArea,"rotor disk area",20.0);
    positive(c.rotorEfficiency,"rotor efficiency",1.0);
    positive(c.batteryWh,"battery capacity",100000.0);
    if (!std::isfinite(c.reserveFraction) || c.reserveFraction<0 || c.reserveFraction>=1)
        throw std::invalid_argument("battery reserve must be in [0,1)");
    positive(c.deliveryRadius,"delivery radius",1000.0);
    positive(c.minSeparation,"minimum separation",1000.0);
    if (!std::isfinite(c.sourceNoiseDb) || c.sourceNoiseDb<0 || c.sourceNoiseDb>150 || !finite(c.noiseObserver))
        throw std::invalid_argument("noise source and observer must be finite and in range");
    if (c.noFlyZones.size()>100) throw std::invalid_argument("at most 100 no-fly rectangles are supported");
    for (const auto& zone:c.noFlyZones) {
        if (!std::isfinite(zone.minX) || !std::isfinite(zone.minY) || !std::isfinite(zone.maxX)
            || !std::isfinite(zone.maxY) || zone.minX>=zone.maxX || zone.minY>=zone.maxY
            || std::max({std::abs(zone.minX),std::abs(zone.minY),std::abs(zone.maxX),std::abs(zone.maxY)})>10000)
            throw std::invalid_argument("no-fly rectangles must have finite ordered bounds within 10 km");
    }
}

SimulationResult runSimulation(const Config& config, bool recordTrace) {
    validateConfig(config);
    SimulationResult result;
    result.requested = config.deliveries;
    result.missions.reserve(static_cast<std::size_t>(config.deliveries));
    HubScheduler hub(config.deliveries);
    WindField wind(config);
    std::vector<Aircraft> aircraft;
    aircraft.reserve(static_cast<std::size_t>(config.drones));
    for (int id=0; id<config.drones; ++id) aircraft.emplace_back(id, config);
    std::vector<FallingPayload> payloads;
    double noiseExposure=0.0;
    std::set<std::pair<int,int>> previousConflicts;
    const auto maximumSteps=static_cast<std::size_t>(std::ceil(config.maxTime/config.dt));

    for (std::size_t step=0; step<maximumSteps; ++step) {
        const double time=step*config.dt;
        const double dt=std::min(config.dt, config.maxTime-time);
        wind.step(dt);
        std::vector<Vec3> positionsBefore;
        positionsBefore.reserve(aircraft.size());
        for (const auto& drone:aircraft) positionsBefore.push_back(drone.state.position);

        // Arrivals already waiting (or now reaching their holding point) have
        // priority over fresh launches. Oldest hold time breaks contention;
        // retaining the token until docking avoids launch-induced starvation.
        for (auto& drone:aircraft) {
            if (drone.phase==Phase::Returning && routeDone(drone)) {
                drone.phase=Phase::Holding;
                drone.holdingSince=time;
                drone.controller.reset();
            }
        }
        const bool arrivalsWaiting=std::any_of(aircraft.begin(),aircraft.end(),
            [](const Aircraft& drone){return drone.phase==Phase::Holding;});
        if (!hub.owner()) {
            Aircraft* waiting=nullptr;
            for (auto& drone:aircraft) {
                if (drone.phase==Phase::Holding && corridorClear(drone,aircraft,config)
                    && (!waiting || drone.holdingSince<waiting->holdingSince))
                    waiting=&drone;
            }
            if (waiting) hub.acquire(waiting->id);
        }
        // FIFO dispatch requires exclusive access to the hub corridor. Each
        // aircraft retains battery energy across successive missions.
        for (auto& drone:aircraft) {
            if (drone.phase != Phase::Parked || drone.disabled || hub.empty() || arrivalsWaiting
                || !corridorClear(drone,aircraft,config) || !hub.acquire(drone.id)) continue;
            const auto next=hub.nextJob();
            if (!next) { hub.release(drone.id); continue; }
            const double angle=2*pi*(*next)/config.deliveries;
            MissionResult mission;
            mission.jobId=*next; mission.droneId=drone.id;
            mission.destination={config.spokeRadius*std::cos(angle),config.spokeRadius*std::sin(angle),0};
            mission.launchedAt=time;
            drone.releasePoint={mission.destination.x,mission.destination.y,drone.altitude};
            const Vec3 drift=predictPayloadDrift(drone.altitude,{},wind.forecastAt(drone.releasePoint),
                                                 config.payloadMass,config.payloadDragArea);
            drone.releasePoint.x-=drift.x; drone.releasePoint.y-=drift.y;
            const Vec3 cruiseStart{drone.pad.x,drone.pad.y,drone.altitude};
            const auto outbound=planRoute(cruiseStart,drone.releasePoint,config);
            const auto inbound=planRoute(drone.releasePoint,drone.holdingPoint(),config);
            const auto approach=planRoute(drone.holdingPoint(),cruiseStart,config);
            const double distance=routeDistance(cruiseStart,outbound)
                                  +routeDistance(drone.releasePoint,inbound)
                                  +routeDistance(drone.holdingPoint(),approach);
            const auto energy=assessMissionEnergy(drone.state.batteryWh,distance,drone.altitude,
                                                   config.vehicleMass+config.payloadMass,config);
            const double launchWind=horizontalNorm(wind.at({drone.pad.x,drone.pad.y,drone.altitude}));
            if (launchWind>config.maxLaunchWind) mission.outcome="rejected_weather";
            else if (config.maxThrust < 1.2*(config.vehicleMass+config.payloadMass)*gravity)
                mission.outcome="rejected_thrust_margin";
            else if (!energy.sufficient) mission.outcome="rejected_energy";
            else if (outbound.empty() || inbound.empty() || approach.empty()) mission.outcome="rejected_no_route";
            if (!mission.outcome.empty()) {
                ++result.rejected;
                result.missions.push_back(mission);
                hub.release(drone.id);
                continue;
            }
            drone.mission=static_cast<int>(result.missions.size());
            mission.outcome="in_flight";
            result.missions.push_back(mission);
            drone.payloadAttached=true; drone.emergency=false; drone.offHubLanding=false;
            drone.energyAtLaunch=drone.state.energyWh;
            drone.phase=Phase::Ascending;
            drone.route={cruiseStart}; drone.waypoint=0;
            drone.plannedReturnRoute=inbound;
            drone.controller.reset();
        }

        std::vector<bool> activeDuringInterval;
        activeDuringInterval.reserve(aircraft.size());
        for (const auto& drone:aircraft) activeDuringInterval.push_back(drone.phase!=Phase::Parked);

        double totalIntensity=0.0;
        for (auto& drone:aircraft) {
            if (drone.phase == Phase::Parked) continue;
            auto& mission=result.missions[static_cast<std::size_t>(drone.mission)];
            if (drone.phase==Phase::Ascending && routeDone(drone)) {
                drone.phase=Phase::Outbound;
                if (!setRoute(drone,drone.releasePoint,config))
                    emergencyLanding(drone,mission,result,"no_route_emergency_landing");
            }
            if (drone.phase==Phase::Outbound && horizontalNorm(drone.filter.state().position-drone.pad)>25.0
                && hub.owner()==std::optional<int>(drone.id)) hub.release(drone.id);
            if (drone.phase==Phase::Outbound && routeDone(drone)) {
                drone.phase=Phase::Releasing;
                drone.controller.reset();
            }
            if (drone.phase==Phase::Releasing && nearTarget(drone,drone.releasePoint,0.7,0.8)) {
                PayloadState parcel;
                parcel.position=drone.state.position; parcel.velocity=drone.state.velocity;
                parcel.mass=config.payloadMass; parcel.dragArea=config.payloadDragArea;
                payloads.push_back({parcel,drone.mission,time});
                mission.releasedAt=time;
                drone.payloadAttached=false;
                drone.phase=Phase::Returning;
                if (!setRoute(drone,drone.holdingPoint(),config))
                    emergencyLanding(drone,mission,result,"no_route_emergency_landing");
            }
            if (drone.phase==Phase::Returning && routeDone(drone)) {
                drone.phase=Phase::Holding;
                drone.holdingSince=time;
                drone.controller.reset();
            }
            if (drone.phase==Phase::Holding && hub.owner()==std::optional<int>(drone.id)) {
                drone.phase=Phase::Landing;
                // Horizontal A* approach remains at the assigned lane before
                // descending vertically over the pad. A diagonal shortcut
                // would bypass no-fly-column checks.
                if (!setRoute(drone,{drone.pad.x,drone.pad.y,drone.altitude},config)) {
                    emergencyLanding(drone,mission,result,"no_route_emergency_landing");
                } else {
                    drone.route.push_back(drone.pad);
                }
            }
            const Vec3 landingTarget=drone.offHubLanding?drone.route.back():drone.pad;
            const bool powerlessGroundContact=drone.offHubLanding && drone.state.batteryWh<=0
                                               && drone.state.position.z<=0.0;
            if (drone.phase==Phase::Landing && (powerlessGroundContact ||
                (drone.state.position.z<0.25 && horizontalNorm(drone.state.position-landingTarget)<0.8
                 && norm(drone.state.velocity)<1.0))) {
                drone.state.position=powerlessGroundContact
                    ? Vec3{drone.state.position.x,drone.state.position.y,0.0}:landingTarget;
                drone.state.velocity={}; drone.state.acceleration={};
                drone.phase=Phase::Parked;
                mission.completedAt=time;
                mission.energyWh=drone.state.energyWh-drone.energyAtLaunch;
                mission.completed=mission.delivered && !drone.offHubLanding;
                if (powerlessGroundContact) mission.outcome="battery_exhausted_ground_contact";
                if (!drone.offHubLanding)
                    mission.outcome=drone.emergency ? "reserve_return" : (mission.withinRadius ? "delivered" : "delivery_miss");
                if (hub.owner()==std::optional<int>(drone.id)) hub.release(drone.id);
                drone.payloadAttached=false;
                // Reset state estimate at the instrumented hub docking station.
                drone.filter=NavigationFilter(drone.state.position,config,config.seed+104729u*(static_cast<unsigned>(drone.id)+1u)
                                                +static_cast<unsigned>(mission.jobId)*65537u);
                continue;
            }

            // Conservative return energy includes descent and reserve; this is
            // checked before releasing cargo, not after exhausting the pack.
            if (!drone.emergency && (drone.phase==Phase::Outbound || drone.phase==Phase::Releasing)) {
                const Vec3 navigationPosition=drone.filter.state().position;
                const auto returnEnergy=assessMissionEnergy(drone.state.batteryWh,
                    horizontalNorm(navigationPosition-drone.pad)+50.0,
                    std::max(0.0,navigationPosition.z),
                    config.vehicleMass+(drone.payloadAttached?config.payloadMass:0.0),config);
                if (!returnEnergy.sufficient) {
                    drone.emergency=true; ++result.emergencyReturns;
                    drone.phase=Phase::Returning;
                    if (!setRoute(drone,drone.holdingPoint(),config))
                        emergencyLanding(drone,mission,result,"no_route_emergency_landing");
                    if (hub.owner()==std::optional<int>(drone.id)) hub.release(drone.id);
                }
            }
            const Vec3 target=routeTarget(drone);
            const auto command=drone.controller.command(target,drone.filter.state(),
                config.vehicleMass+(drone.payloadAttached?config.payloadMass:0.0),
                wind.forecastAt(drone.filter.state().position),dt);
            if (command.saturated) ++result.saturationSteps;
            const Vec3 actualWind=wind.at(drone.state.position);
            const double power=advanceVehicle(drone.state,command.force,actualWind,
                config.vehicleMass+(drone.payloadAttached?config.payloadMass:0.0),dt,config);
            const Estimate estimate=drone.filter.update(drone.state,dt);
            if (drone.state.batteryWh<=0 && !drone.offHubLanding)
                emergencyLanding(drone,mission,result,"battery_exhausted_emergency_landing");
            result.maxNavigationError=std::max(result.maxNavigationError,norm(estimate.position-drone.state.position));
            if (drone.state.batteryWh>0.0) {
                const double level=noiseDb(power,norm(drone.state.position-config.noiseObserver),config);
                totalIntensity+=std::pow(10.0,level/10.0);
            }
            if (recordTrace) result.trace.push_back({time+dt,drone.id,phaseName(drone.phase),
                drone.state.position,estimate.position,drone.state.velocity,actualWind,
                drone.state.batteryWh,norm(command.force),drone.payloadAttached,hub.owner().value_or(-1)});
        }

        for (auto& parcel:payloads) {
            if (parcel.state.impacted) continue;
            advancePayload(parcel.state,wind.at(parcel.state.position),dt);
            if (parcel.state.impacted) {
                auto& mission=result.missions[static_cast<std::size_t>(parcel.mission)];
                mission.delivered=true;
                mission.impact=parcel.state.impact;
                mission.impactAt=parcel.releaseTime+parcel.state.flightTime;
                mission.impactError=horizontalNorm(mission.impact-mission.destination);
                mission.withinRadius=mission.impactError<=config.deliveryRadius;
                if (mission.completedAt>0 && mission.outcome!="no_route_emergency_landing"
                    && mission.outcome!="battery_exhausted_emergency_landing"
                    && mission.outcome!="battery_exhausted_ground_contact") {
                    mission.completed=true;
                    mission.outcome=mission.withinRadius?"delivered":"delivery_miss";
                }
            }
        }

        std::set<std::pair<int,int>> conflicts;
        for (std::size_t a=0; a<aircraft.size(); ++a) for (std::size_t b=a+1; b<aircraft.size(); ++b) {
            if (!activeDuringInterval[a] || !activeDuringInterval[b]) continue;
            // Closest relative approach over the whole step, assuming linear
            // interpolation between integrated endpoints. This improves on
            // endpoint sampling; it is not a swept-rotor collision proof.
            const Vec3 relativeStart=positionsBefore[a]-positionsBefore[b];
            const Vec3 relativeMotion=(aircraft[a].state.position-positionsBefore[a])
                                     -(aircraft[b].state.position-positionsBefore[b]);
            const double motionSquared=relativeMotion.x*relativeMotion.x
                +relativeMotion.y*relativeMotion.y+relativeMotion.z*relativeMotion.z;
            const double projection=relativeStart.x*relativeMotion.x
                +relativeStart.y*relativeMotion.y+relativeStart.z*relativeMotion.z;
            const double fraction=motionSquared>0.0?std::clamp(-projection/motionSquared,0.0,1.0):0.0;
            const double separation=norm(relativeStart+relativeMotion*fraction);
            result.minimumSeparation=std::min(result.minimumSeparation,separation);
            if (separation<config.minSeparation) {
                const std::pair<int,int> pair{static_cast<int>(a),static_cast<int>(b)};
                conflicts.insert(pair);
                if (!previousConflicts.count(pair)) ++result.separationEvents;
            }
        }
        previousConflicts=std::move(conflicts);
        if (totalIntensity>0) {
            result.maxNoiseDb=std::max(result.maxNoiseDb,10.0*std::log10(totalIntensity));
            noiseExposure+=totalIntensity*dt;
        }
        result.duration=time+dt;
        const bool aircraftIdle=std::all_of(aircraft.begin(),aircraft.end(),[](const Aircraft& d){return d.phase==Phase::Parked;});
        const bool parcelsLanded=std::all_of(payloads.begin(),payloads.end(),[](const FallingPayload& p){return p.state.impacted;});
        if (hub.empty() && aircraftIdle && parcelsLanded) break;
        if (!hub.empty() && aircraftIdle && std::all_of(aircraft.begin(),aircraft.end(),
            [](const Aircraft& d){return d.disabled;})) {
            while (const auto job=hub.nextJob()) {
                MissionResult mission;
                mission.jobId=*job; mission.droneId=-1;
                const double angle=2*pi*(*job)/config.deliveries;
                mission.destination={config.spokeRadius*std::cos(angle),config.spokeRadius*std::sin(angle),0};
                mission.outcome="rejected_no_serviceable_aircraft";
                ++result.rejected;
                result.missions.push_back(mission);
            }
            if (parcelsLanded) break;
        }
        if (step+1==maximumSteps) result.timedOut=true;
    }
    // Partial flights retain consumed energy, and unlaunched requests are
    // represented explicitly rather than disappearing at the horizon.
    for (const auto& drone:aircraft) {
        if (drone.phase!=Phase::Parked && drone.mission>=0)
            result.missions[static_cast<std::size_t>(drone.mission)].energyWh=
                drone.state.energyWh-drone.energyAtLaunch;
    }
    while (const auto job=hub.nextJob()) {
        MissionResult mission;
        mission.jobId=*job; mission.droneId=-1;
        const double angle=2*pi*(*job)/config.deliveries;
        mission.destination={config.spokeRadius*std::cos(angle),config.spokeRadius*std::sin(angle),0};
        mission.outcome="not_dispatched_timeout";
        result.missions.push_back(mission);
    }
    for (auto& mission:result.missions) {
        result.delivered+=mission.delivered?1:0;
        result.completed+=mission.completed?1:0;
        result.withinRadius+=mission.withinRadius?1:0;
        if (mission.outcome=="in_flight") mission.outcome="simulation_timeout";
    }
    for (const auto& drone:aircraft) result.energyWh+=drone.state.energyWh;
    if (noiseExposure>0) {
        result.noiseSelDb=10.0*std::log10(noiseExposure);
        result.noiseLaeqDb=10.0*std::log10(noiseExposure/result.duration);
    }
    return result;
}
} // namespace sim
