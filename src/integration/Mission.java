package integration;

public record Mission(int jobId, int droneId,
                      double destinationX, double destinationY, double destinationZ,
                      Double impactX, Double impactY, Double impactZ,
                      boolean delivered, boolean completed, boolean withinRadius,
                      Double impactError, double energyWh, String outcome,
                      double launchedAt, double releasedAt, double impactAt, double completedAt) {
    public GeoCoordinate destinationAt(Hub hub) {
        return hub.locationOf(destinationX, destinationY, destinationZ);
    }
    public GeoCoordinate impactAt(Hub hub) {
        return delivered ? hub.locationOf(impactX, impactY, impactZ) : null;
    }
}
