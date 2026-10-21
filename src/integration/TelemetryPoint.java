package integration;

public record TelemetryPoint(double time, int droneId, String phase,
                             double x, double y, double z,
                             double estimatedX, double estimatedY, double estimatedZ,
                             double vx, double vy, double vz,
                             double windX, double windY, double windZ,
                             double batteryWh, double thrustN,
                             boolean payloadAttached, int hubOwner) {
    public double speed() { return Math.sqrt(vx*vx + vy*vy + vz*vz); }
    public double navigationError() {
        return Math.sqrt(Math.pow(x-estimatedX, 2) + Math.pow(y-estimatedY, 2)
                + Math.pow(z-estimatedZ, 2));
    }
    public GeoCoordinate geographic(Hub hub) { return hub.locationOf(x, y, z); }
}
