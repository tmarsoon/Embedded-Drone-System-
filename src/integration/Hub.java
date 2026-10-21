package integration;

import java.util.Objects;

public record Hub(String id, String name, double latitude, double longitude) {
    public Hub {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(name, "name");
        if (id.isBlank() || name.isBlank() || !Double.isFinite(latitude)
                || !Double.isFinite(longitude) || Math.abs(latitude) >= 89
                || Math.abs(longitude) > 180) {
            throw new IllegalArgumentException("Invalid hub definition");
        }
    }

    /** Local tangent-plane approximation for metre-scale missions at this hub. */
    public GeoCoordinate locationOf(double east, double north, double altitude) {
        if (!Double.isFinite(east) || !Double.isFinite(north) || !Double.isFinite(altitude)
                || Math.hypot(east, north) > 10_000) {
            throw new IllegalArgumentException("Georeferencing is limited to a 10 km local mission");
        }
        double radius = 6_378_137.0;
        double lat = latitude + Math.toDegrees(north / radius);
        double lon = longitude + Math.toDegrees(east / (radius * Math.cos(Math.toRadians(latitude))));
        lon = ((lon + 180) % 360 + 360) % 360 - 180;
        return new GeoCoordinate(lat, lon, altitude);
    }

    @Override public String toString() { return name; }
}
