package integration;

/** Parameters exposed by the desktop dashboard; the native CLI retains its full option set. */
public record SimulationRequest(String hubId, int drones, int deliveries, long seed,
                                double wind, double gust, double payload, double gpsNoise) {
    public SimulationRequest {
        HubCatalog.byId(hubId);
        if (drones < 1 || drones > 32 || deliveries < 1 || deliveries > 1000
                || seed < 0 || seed > 0xffff_ffffL) {
            throw new IllegalArgumentException("Use 1–32 aircraft, 1–1000 deliveries and a uint32 seed");
        }
        range(wind, 0, 60, "wind");
        range(gust, 0, 20, "gust");
        range(payload, Double.MIN_NORMAL, 20, "payload mass");
        range(gpsNoise, 0, 100, "GPS noise");
    }

    private static void range(double value, double minimum, double maximum, String label) {
        if (!Double.isFinite(value) || value < minimum || value > maximum) {
            throw new IllegalArgumentException(label + " is outside the supported model range");
        }
    }
    public static SimulationRequest defaults() { return defaults("offutt"); }
    public static SimulationRequest defaults(String hubId) {
        return new SimulationRequest(hubId, 3, 6, 42, 3, 0.8, 0.3, 0.25);
    }
}
