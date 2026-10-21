package integration;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/** Exercises the actual child executable and the Java/native data boundary. */
class BridgeTest {
    @TempDir Path temporary;
    private NativeSimulator simulator() throws IOException { return new NativeSimulator(); }
    private SimulationRun run(String directory) throws Exception {
        try (NativeSimulator simulator = simulator()) {
            return simulator.run(SimulationRequest.defaults(), temporary.resolve(directory), Duration.ofSeconds(30));
        }
    }

    @Test void actualNativeFlightLoadsTelemetryAndGeographicResultsInPathsWithSpaces() throws Exception {
        SimulationRun run = run("run with spaces");
        assertEquals(6, run.metrics().get("completed").getAsInt());
        assertEquals(6, run.metrics().get("within_radius").getAsInt());
        assertFalse(run.telemetry().isEmpty());
        assertEquals(3, run.byDrone().size());
        assertTrue(run.telemetry().stream().anyMatch(TelemetryPoint::payloadAttached));
        assertTrue(run.telemetry().stream().anyMatch(p -> !p.payloadAttached()));
        assertTrue(run.missions().stream().allMatch(Mission::completed));
        JsonObject report = JsonParser.parseString(Files.readString(run.directory().resolve("integrated-report.json"))).getAsJsonObject();
        assertEquals("offutt", report.getAsJsonObject("hub").get("id").getAsString());
        assertEquals(6, report.getAsJsonArray("geographic_missions").size());
        assertEquals(1, report.get("integration_schema_version").getAsInt());
    }

    @Test void localProjectionRespectsEastNorthUpAndRejectsGlobalFlightAssumptions() {
        Hub hub = HubCatalog.byId("north-island");
        GeoCoordinate origin = hub.locationOf(0, 0, 40);
        assertEquals(hub.latitude(), origin.latitude(), 1e-10);
        assertEquals(hub.longitude(), origin.longitude(), 1e-10);
        assertTrue(hub.locationOf(100, 0, 40).longitude() > origin.longitude());
        assertTrue(hub.locationOf(0, 100, 40).latitude() > origin.latitude());
        assertThrows(IllegalArgumentException.class, () -> hub.locationOf(10001, 0, 40));
        assertThrows(IllegalArgumentException.class, () -> HubCatalog.byId("unknown"));
    }

    @Test void scenarioValidationRejectsNonfiniteAndOutOfRangeInputs() {
        assertThrows(IllegalArgumentException.class, () -> new SimulationRequest("offutt", 0, 6, 42, 3, .8, .3, .25));
        assertThrows(IllegalArgumentException.class, () -> new SimulationRequest("offutt", 3, 6, -1, 3, .8, .3, .25));
        assertThrows(IllegalArgumentException.class, () -> new SimulationRequest("offutt", 3, 6, 42, Double.NaN, .8, .3, .25));
        assertThrows(IllegalArgumentException.class, () -> new SimulationRequest("offutt", 3, 6, 42, 3, .8, 0, .25));
    }

    @Test void csvBoundaryHandlesEscapesAndRejectsBrokenQuotes() throws IOException {
        assertEquals(List.of("a,b", "a\"b", ""), Csv.parseLine("\"a,b\",\"a\"\"b\","));
        assertThrows(IOException.class, () -> Csv.parseLine("\"unterminated"));
        assertThrows(IOException.class, () -> Csv.parseLine("\"a\"garbage,b"));
    }

    @Test void weatherRejectionsRemainVisibleInsteadOfBecomingFakeSuccessfulFlights() throws Exception {
        var request = new SimulationRequest("gregg-adams", 3, 6, 42, 20, .8, .3, .25);
        try (NativeSimulator simulator = simulator()) {
            SimulationRun run = simulator.run(request, temporary.resolve("weather"), Duration.ofSeconds(30));
            assertEquals(6, run.metrics().get("rejected").getAsInt());
            assertEquals(0, run.metrics().get("completed").getAsInt());
            assertTrue(run.telemetry().isEmpty());
            assertTrue(run.missions().stream().allMatch(m -> !m.delivered() && m.impactX() == null));
        }
    }

    @Test void existingOutputIsPreserved() throws Exception {
        Path directory = temporary.resolve("existing");
        Files.createDirectory(directory);
        Path marker = directory.resolve("user.txt");
        Files.writeString(marker, "preserve");
        try (NativeSimulator simulator = simulator()) {
            assertThrows(IOException.class, () -> simulator.run(SimulationRequest.defaults(), directory, Duration.ofSeconds(30)));
        }
        assertEquals("preserve", Files.readString(marker));
    }

    @Test void schemaAndMissionAccountingTamperingAreRejected() throws Exception {
        SimulationRun run = run("invalid summary");
        Path file = run.directory().resolve("summary.json");
        String original = Files.readString(file);
        JsonObject summary = JsonParser.parseString(original).getAsJsonObject();
        summary.addProperty("schema_version", 2);
        Files.writeString(file, summary.toString());
        assertThrows(IOException.class, () -> NativeSimulator.load(SimulationRequest.defaults(), run.directory()));
        summary = JsonParser.parseString(original).getAsJsonObject();
        summary.getAsJsonObject("metrics").addProperty("completed", 7);
        Files.writeString(file, summary.toString());
        assertThrows(IOException.class, () -> NativeSimulator.load(SimulationRequest.defaults(), run.directory()));
    }

    @Test void duplicateOrNonfiniteTelemetryIsRejected() throws Exception {
        SimulationRun run = run("invalid telemetry");
        Path file = run.directory().resolve("telemetry.csv");
        List<String> rows = Files.readAllLines(file);
        assertTrue(rows.size() > 2);
        Files.writeString(file, rows.get(0) + "\n" + rows.get(1) + "\n" + rows.get(1) + "\n");
        assertThrows(IOException.class, () -> NativeSimulator.load(SimulationRequest.defaults(), run.directory()));
        String[] fields = rows.get(1).split(",", -1);
        fields[3] = "NaN";
        Files.writeString(file, rows.get(0) + "\n" + String.join(",", fields) + "\n");
        assertThrows(IOException.class, () -> NativeSimulator.load(SimulationRequest.defaults(), run.directory()));
    }

    @Test void timeoutTerminatesTheActualNativeChild() throws Exception {
        var request = new SimulationRequest("offutt", 32, 1000, 42, 3, .8, .3, .25);
        try (NativeSimulator simulator = simulator()) {
            IOException error = assertThrows(IOException.class, () -> simulator.run(request, temporary.resolve("timeout"), Duration.ofMillis(1)));
            assertTrue(error.getMessage().contains("wall-time"), error.getMessage());
            assertFalse(simulator.isRunning());
        }
    }

    @Test void cancellationStopsTheNativeJobAndReleasesTheBridge() throws Exception {
        var request = new SimulationRequest("offutt", 32, 1000, 42, 3, .8, .3, .25);
        var executor = Executors.newSingleThreadExecutor();
        try (NativeSimulator simulator = simulator()) {
            var future = executor.submit(() -> simulator.run(request, temporary.resolve("cancel"), Duration.ofSeconds(30)));
            long limit = System.nanoTime() + TimeUnit.SECONDS.toNanos(3);
            while (!simulator.isRunning() && System.nanoTime() < limit) Thread.onSpinWait();
            assertTrue(simulator.isRunning());
            simulator.cancel();
            ExecutionException error = assertThrows(ExecutionException.class, () -> future.get(5, TimeUnit.SECONDS));
            assertInstanceOf(IOException.class, error.getCause());
            assertFalse(simulator.isRunning());
        } finally { executor.shutdownNow(); }
    }
}
