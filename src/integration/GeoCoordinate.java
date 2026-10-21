package integration;

/** Geographic context for a local east/north/up mission, not a global flight model. */
public record GeoCoordinate(double latitude, double longitude, double altitude) {
    public GeoCoordinate {
        if (!Double.isFinite(latitude) || !Double.isFinite(longitude)
                || !Double.isFinite(altitude) || latitude < -90 || latitude > 90
                || longitude < -180 || longitude > 180) {
            throw new IllegalArgumentException("Invalid geographic coordinate");
        }
    }
}
