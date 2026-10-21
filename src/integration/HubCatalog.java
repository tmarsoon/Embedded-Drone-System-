package integration;

import java.util.List;

/** The four geographic markers from the uploaded Java template. */
public final class HubCatalog {
    private HubCatalog() {}
    private static final List<Hub> HUBS = List.of(
            new Hub("offutt", "Offutt Air Force Base", 41.118, -95.912),
            new Hub("north-island", "Naval Air Station North Island", 32.699, -117.215),
            new Hub("gregg-adams", "Fort Gregg-Adams", 37.239, -77.338),
            new Hub("marine-district", "1st Marine Corps District", 40.7, -73.6));

    public static List<Hub> all() { return HUBS; }
    public static Hub byId(String id) {
        return HUBS.stream().filter(h -> h.id().equals(id)).findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Unknown hub: " + id));
    }
}
