package integration;

import com.google.gson.JsonObject;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public record SimulationRun(Hub hub, Path directory, JsonObject summary,
                            List<TelemetryPoint> telemetry, List<Mission> missions) {
    public SimulationRun {
        directory = directory.toAbsolutePath().normalize();
        summary = summary.deepCopy();
        telemetry = List.copyOf(telemetry);
        missions = List.copyOf(missions);
    }
    public JsonObject metrics() { return summary.getAsJsonObject("metrics"); }
    public double duration() { return metrics().get("duration_s").getAsDouble(); }
    public Map<Integer, List<TelemetryPoint>> byDrone() {
        Map<Integer, List<TelemetryPoint>> result = new LinkedHashMap<>();
        for (TelemetryPoint point : telemetry) {
            result.computeIfAbsent(point.droneId(), unused -> new ArrayList<>()).add(point);
        }
        result.replaceAll((id, points) -> List.copyOf(points));
        return Collections.unmodifiableMap(result);
    }
}
