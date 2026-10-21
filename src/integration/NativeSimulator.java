package integration;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.Strictness;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.math.BigDecimal;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/** Runs the authoritative C++ model off the JavaFX thread and validates its output contract. */
public final class NativeSimulator implements AutoCloseable {
    private static final Gson JSON = new GsonBuilder().setStrictness(Strictness.STRICT).create();
    private static final String HEADER = "time_s,drone_id,phase,x_m,y_m,z_m,estimated_x_m,estimated_y_m,estimated_z_m,"
            + "vx_mps,vy_mps,vz_mps,wind_x_mps,wind_y_mps,wind_z_mps,battery_wh,thrust_n,payload_attached,hub_owner";
    private static final Set<String> PHASES = Set.of("parked", "ascending", "outbound", "releasing", "returning", "holding", "landing");
    private static final long MAX_SUMMARY_BYTES = 16L * 1024 * 1024;
    private static final long MAX_TRACE_BYTES = 512L * 1024 * 1024;
    private final Path projectRoot;
    private final AtomicReference<Process> active = new AtomicReference<>();
    private final AtomicBoolean running = new AtomicBoolean();
    private final Object lifecycle = new Object();
    private volatile boolean cancellationRequested;

    public NativeSimulator() throws IOException { this(findProjectRoot()); }
    public NativeSimulator(Path projectRoot) throws IOException {
        this.projectRoot = projectRoot.toRealPath();
        if (!Files.isDirectory(this.projectRoot.resolve("native"))
                || !Files.isDirectory(this.projectRoot.resolve("src"))) {
            throw new IOException("Project root must contain native/ and src/: " + projectRoot);
        }
    }
    public Path projectRoot() { return projectRoot; }
    public boolean isRunning() { return running.get(); }

    public static Path findProjectRoot() throws IOException {
        String override = System.getProperty("drone.project.root");
        if (override != null && !override.isBlank()) return Path.of(override).toAbsolutePath().normalize();
        Path current = Path.of(System.getProperty("user.dir")).toAbsolutePath().normalize();
        for (Path path = current; path != null; path = path.getParent()) {
            if (Files.isDirectory(path.resolve("native")) && Files.isDirectory(path.resolve("src"))) return path;
        }
        throw new IOException("Cannot locate project root; launch from the project folder or set -Ddrone.project.root=...");
    }

    public Path discoverBinary() throws IOException {
        String explicit = System.getProperty("drone.native.path");
        if (explicit == null || explicit.isBlank()) explicit = System.getenv("DRONE_SIM_BINARY");
        if (explicit != null && !explicit.isBlank()) {
            Path path = Path.of(explicit);
            if (!path.isAbsolute()) path = projectRoot.resolve(path);
            return requireExecutable(path.normalize());
        }
        for (String name : List.of("native/build/drone_sim", "native/build/drone_sim.exe",
                "native/build/Release/drone_sim.exe", "native/build/Debug/drone_sim.exe")) {
            Path path = projectRoot.resolve(name);
            if (Files.isRegularFile(path) && Files.isExecutable(path)) return path.toRealPath();
        }
        throw new IOException("Native flight engine is not built. Run 'make -C native -j4' or build native/ with CMake, "
                + "then reopen the mission. Set -Ddrone.native.path=... for a custom executable.");
    }
    private static Path requireExecutable(Path path) throws IOException {
        if (!Files.isRegularFile(path) || !Files.isExecutable(path)) throw new IOException("Native executable is unavailable: " + path);
        return path.toRealPath();
    }

    public SimulationRun run(SimulationRequest request, Duration timeout) throws IOException, InterruptedException {
        Path runs = projectRoot.resolve("runs");
        Files.createDirectories(runs);
        return run(request, Files.createTempDirectory(runs, "mission-"), timeout);
    }

    public SimulationRun run(SimulationRequest request, Path output, Duration timeout) throws IOException, InterruptedException {
        if (request == null || timeout == null || timeout.isZero() || timeout.isNegative()
                || timeout.compareTo(Duration.ofHours(1)) > 0) {
            throw new IllegalArgumentException("A request and positive timeout of at most one hour are required");
        }
        synchronized (lifecycle) {
            if (!running.compareAndSet(false, true)) throw new IllegalStateException("This simulator already has a running mission");
            cancellationRequested = false;
        }
        Process process = null;
        try {
            if (Thread.currentThread().isInterrupted()) throw new InterruptedException("Mission cancelled before launch");
            Path executable = discoverBinary();
            Path directory = output.toAbsolutePath().normalize();
            Files.createDirectories(directory);
            try (var files = Files.list(directory)) {
                if (files.findAny().isPresent()) throw new IOException("Use a new empty output folder; existing run files are preserved: " + directory);
            }
            List<String> command = new ArrayList<>(List.of(executable.toString(), "--drones", ""+request.drones(),
                    "--deliveries", ""+request.deliveries(), "--seed", ""+request.seed(),
                    "--wind", ""+request.wind(), "--gust", ""+request.gust(),
                    "--payload", ""+request.payload(), "--gps-noise", ""+request.gpsNoise(),
                    "--output", directory.toString(), "--trace"));
            JsonObject invocation = new JsonObject();
            invocation.addProperty("integration_schema_version", 1);
            invocation.addProperty("hub_id", request.hubId());
            invocation.add("command", JSON.toJsonTree(command));
            Files.writeString(directory.resolve("invocation.json"), JSON.toJson(invocation)+"\n", StandardCharsets.UTF_8);
            ProcessBuilder builder = new ProcessBuilder(command).directory(projectRoot.toFile());
            builder.redirectOutput(directory.resolve("native-stdout.json").toFile());
            builder.redirectError(directory.resolve("native-stderr.log").toFile());
            process = builder.start();
            active.set(process);
            if (cancellationRequested) {
                terminate(process);
                throw new IOException("Mission cancelled; partial output retained in " + directory);
            }
            boolean exited = process.waitFor(Math.max(1, timeout.toMillis()), TimeUnit.MILLISECONDS);
            if (!exited) {
                terminate(process);
                throw new IOException("Native mission exceeded wall-time limit " + timeout + "; partial output retained in " + directory);
            }
            if (cancellationRequested) throw new IOException("Mission cancelled; partial output retained in " + directory);
            if (process.exitValue() != 0) {
                throw new IOException("Native engine exited " + process.exitValue() + ": "
                        + logTail(directory.resolve("native-stderr.log")));
            }
            SimulationRun result = load(request, directory);
            Files.writeString(directory.resolve("integrated-report.json"), georeferencedReport(result)+"\n", StandardCharsets.UTF_8);
            return result;
        } catch (InterruptedException interrupted) {
            if (process != null) terminate(process);
            Thread.currentThread().interrupt();
            throw interrupted;
        } finally {
            if (process != null && process.isAlive()) terminate(process);
            active.set(null);
            running.set(false);
        }
    }

    public void cancel() {
        Process process;
        synchronized (lifecycle) {
            cancellationRequested = true;
            process = active.get();
        }
        if (process != null) terminate(process);
    }
    @Override public void close() { cancel(); }

    private static void terminate(Process process) {
        // The executable is normally single-process; descendants are included so
        // custom launch wrappers cannot leave a cancelled simulation running.
        List<ProcessHandle> children = process.descendants().toList();
        children.forEach(ProcessHandle::destroy);
        process.destroy();
        try {
            if (!process.waitFor(500, TimeUnit.MILLISECONDS)) process.destroyForcibly();
        } catch (InterruptedException interrupted) {
            process.destroyForcibly();
            Thread.currentThread().interrupt();
        }
        children.stream().filter(ProcessHandle::isAlive).forEach(ProcessHandle::destroyForcibly);
    }
    private static String logTail(Path path) throws IOException {
        long size = Files.size(path);
        try (var input = Files.newInputStream(path)) {
            input.skipNBytes(Math.max(0, size-4096));
            return new String(input.readNBytes(4096), StandardCharsets.UTF_8).strip();
        }
    }

    /** Load an already completed run using the same strict boundary checks as a fresh child process. */
    public static SimulationRun load(SimulationRequest request, Path directory) throws IOException {
        try {
            Path summaryPath = directory.resolve("summary.json");
            if (!Files.isRegularFile(summaryPath) || Files.size(summaryPath) > MAX_SUMMARY_BYTES) throw new IOException("Missing or oversized summary.json");
            JsonObject summary;
            try (Reader reader = Files.newBufferedReader(summaryPath, StandardCharsets.UTF_8)) {
                summary = JSON.fromJson(reader, JsonObject.class);
            }
            if (summary == null || integer(summary,"schema_version") != 1
                    || !string(summary,"mode").equals("single")
                    || !string(summary,"coordinate_frame").equals("ENU_m")) throw new IOException("Unsupported native output schema");
            JsonObject config = object(summary,"config"), metrics = object(summary,"metrics");
            if (integer(config,"drones") != request.drones() || integer(config,"deliveries") != request.deliveries()
                    || decimal(config,"seed").longValueExact() != request.seed()
                    || !same(number(config,"wind_speed_mps"),request.wind())
                    || !same(number(config,"gust_sigma_mps"),request.gust())
                    || !same(number(config,"payload_mass_kg"),request.payload())
                    || !same(number(config,"gps_sigma_m"),request.gpsNoise())) throw new IOException("Native output does not match the requested scenario");
            int requested = integer(metrics,"requested"), delivered = integer(metrics,"delivered"), completed = integer(metrics,"completed");
            int accepted = integer(metrics,"within_radius"), rejected = integer(metrics,"rejected");
            if (requested != request.deliveries() || delivered < 0 || delivered > requested
                    || completed < 0 || completed > delivered || accepted < 0 || accepted > delivered
                    || rejected < 0 || rejected > requested || rejected+delivered > requested) throw new IOException("Invalid native mission accounting");
            double duration = number(metrics,"duration_s");
            if (duration <= 0 || number(metrics,"energy_wh") < 0) throw new IOException("Invalid duration or energy");
            for (String field : List.of("minimum_separation_m","max_noise_db","noise_laeq_db","noise_sel_db","max_navigation_error_m")) {
                JsonElement value = required(metrics,field);
                if (!value.isJsonNull()) number(metrics,field);
            }
            for (String field : List.of("separation_events","emergency_returns","saturation_steps")) {
                if (integer(metrics,field)<0) throw new IOException("Negative event count");
            }
            bool(metrics,"timed_out");
            JsonArray rows = required(summary,"missions").getAsJsonArray();
            if (rows.size()!=requested) throw new IOException("Native output omitted mission records");
            List<Mission> missions = new ArrayList<>();
            Set<Integer> identities = new HashSet<>();
            int seenDelivered=0, seenCompleted=0, seenAccepted=0;
            for (JsonElement element : rows) {
                JsonObject row=element.getAsJsonObject();
                int id=integer(row,"job_id"), drone=integer(row,"drone_id");
                if (id<0 || id>=requested || !identities.add(id) || drone < -1 || drone>=request.drones()) throw new IOException("Invalid mission identity");
                boolean hasImpact=bool(row,"delivered"), returned=bool(row,"completed"), inRadius=bool(row,"within_radius");
                if ((returned || inRadius) && !hasImpact) throw new IOException("Unimpacted mission claimed a successful outcome");
                double[] destination=vector(row,"destination_m");
                double[] impact = hasImpact ? vector(row,"impact_m") : null;
                if (!hasImpact && (!required(row,"impact_m").isJsonNull() || !required(row,"impact_error_m").isJsonNull())) throw new IOException("Undelivered mission has fabricated impact data");
                Double error=hasImpact?number(row,"impact_error_m"):null;
                if (error!=null && error<0) throw new IOException("Negative impact error");
                double energy=number(row,"energy_wh");
                if (energy<0) throw new IOException("Negative mission energy");
                String outcome=string(row,"outcome");
                if (!outcome.matches("[a-z_]+")) throw new IOException("Invalid mission outcome");
                missions.add(new Mission(id,drone,destination[0],destination[1],destination[2],
                        impact==null?null:impact[0],impact==null?null:impact[1],impact==null?null:impact[2],
                        hasImpact,returned,inRadius,error,energy,outcome,
                        nonnegative(row,"launched_at_s"),nonnegative(row,"released_at_s"),
                        nonnegative(row,"impact_at_s"),nonnegative(row,"completed_at_s")));
                seenDelivered+=hasImpact?1:0; seenCompleted+=returned?1:0; seenAccepted+=inRadius?1:0;
            }
            if (seenDelivered!=delivered || seenCompleted!=completed || seenAccepted!=accepted) throw new IOException("Mission and aggregate outcomes disagree");
            List<TelemetryPoint> telemetry=readTelemetry(directory.resolve("telemetry.csv"), request.drones(), duration);
            if (delivered>0 && telemetry.isEmpty()) throw new IOException("Delivered missions lack telemetry");
            return new SimulationRun(HubCatalog.byId(request.hubId()),directory,summary,telemetry,missions);
        } catch (IOException exception) {
            throw exception;
        } catch (RuntimeException exception) {
            throw new IOException("Invalid native output: " + exception.getMessage(),exception);
        }
    }

    private static List<TelemetryPoint> readTelemetry(Path path,int drones,double duration) throws IOException {
        if (!Files.isRegularFile(path) || Files.size(path)>MAX_TRACE_BYTES) throw new IOException("Missing or oversized telemetry.csv");
        List<TelemetryPoint> result=new ArrayList<>();
        Map<Integer,Double> previous=new HashMap<>();
        double globalTime=-1;
        try (BufferedReader reader=Files.newBufferedReader(path,StandardCharsets.UTF_8)) {
            if (!HEADER.equals(reader.readLine())) throw new IOException("Unsupported native telemetry columns");
            String line;
            while ((line=reader.readLine())!=null) {
                if (line.length()>8192 || result.size()>=2_000_000) throw new IOException("Telemetry record limit exceeded");
                List<String> cells=Csv.parseLine(line);
                if (cells.size()!=19) throw new IOException("Telemetry column count mismatch");
                double time=csvDouble(cells.get(0));
                int id=csvInt(cells.get(1)), owner=csvInt(cells.get(18));
                String phase=cells.get(2);
                if (id<0 || id>=drones || owner < -1 || owner>=drones || !PHASES.contains(phase)
                        || time<0 || time>duration+1e-8 || time<globalTime-1e-10
                        || time<=previous.getOrDefault(id,-1.0)) throw new IOException("Invalid or unordered native telemetry");
                double[] values=new double[14];
                for (int column=3;column<=16;++column) values[column-3]=csvDouble(cells.get(column));
                if (values[2]<0 || values[12]<0 || values[13]<0) throw new IOException("Negative altitude, battery or thrust");
                if (!cells.get(17).equals("0") && !cells.get(17).equals("1")) throw new IOException("Invalid payload attachment flag");
                result.add(new TelemetryPoint(time,id,phase,values[0],values[1],values[2],
                        values[3],values[4],values[5],values[6],values[7],values[8],
                        values[9],values[10],values[11],values[12],values[13],cells.get(17).equals("1"),owner));
                previous.put(id,time); globalTime=time;
            }
        } catch (NumberFormatException exception) {
            throw new IOException("Invalid telemetry numeric value",exception);
        }
        return result;
    }

    private static double csvDouble(String value) throws IOException {
        if (!value.matches("-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?")) throw new IOException("Invalid telemetry number");
        double result=Double.parseDouble(value);
        if (!Double.isFinite(result)) throw new IOException("Non-finite telemetry value");
        return result;
    }
    private static int csvInt(String value) { return Integer.parseInt(value); }
    private static boolean same(double a,double b) { return Math.abs(a-b)<=1e-9*Math.max(1,Math.max(Math.abs(a),Math.abs(b))); }
    private static JsonElement required(JsonObject object,String key) throws IOException {
        if (!object.has(key)) throw new IOException("Missing native field: " + key);
        return object.get(key);
    }
    private static JsonObject object(JsonObject object,String key) throws IOException { return required(object,key).getAsJsonObject(); }
    private static String string(JsonObject object,String key) throws IOException {
        JsonElement value=required(object,key);
        if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isString()) throw new IOException("Expected native string: " + key);
        return value.getAsString();
    }
    private static BigDecimal decimal(JsonObject object,String key) throws IOException {
        JsonElement value=required(object,key);
        if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()) throw new IOException("Expected native number: " + key);
        return value.getAsBigDecimal();
    }
    private static int integer(JsonObject object,String key) throws IOException { return decimal(object,key).intValueExact(); }
    private static double number(JsonObject object,String key) throws IOException {
        double result=decimal(object,key).doubleValue();
        if (!Double.isFinite(result)) throw new IOException("Non-finite native field: " + key);
        return result;
    }
    private static double nonnegative(JsonObject object,String key) throws IOException {
        double result=number(object,key);
        if (result<0) throw new IOException("Negative native field: " + key);
        return result;
    }
    private static boolean bool(JsonObject object,String key) throws IOException {
        JsonElement value=required(object,key);
        if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isBoolean()) throw new IOException("Expected native boolean: " + key);
        return value.getAsBoolean();
    }
    private static double[] vector(JsonObject object,String key) throws IOException {
        JsonArray array=required(object,key).getAsJsonArray();
        if (array.size()!=3) throw new IOException("Native vector must have three coordinates");
        double[] result=new double[3];
        for (int index=0;index<3;++index) {
            JsonElement value=array.get(index);
            if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()
                    || !Double.isFinite(result[index]=value.getAsDouble())) throw new IOException("Invalid native vector coordinate");
        }
        return result;
    }

    public static String georeferencedReport(SimulationRun run) {
        JsonObject output=new JsonObject();
        output.addProperty("integration_schema_version",1);
        output.add("hub",JSON.toJsonTree(run.hub()));
        output.addProperty("coordinate_model","Local tangent plane, east/north/up metres; geographic context only, not global flight dynamics");
        output.add("flight",run.summary());
        JsonArray missions=new JsonArray();
        for (Mission mission:run.missions()) {
            JsonObject row=new JsonObject();
            row.addProperty("job_id",mission.jobId());
            row.add("destination",JSON.toJsonTree(mission.destinationAt(run.hub())));
            row.add("impact",JSON.toJsonTree(mission.impactAt(run.hub())));
            missions.add(row);
        }
        output.add("geographic_missions",missions);
        return new GsonBuilder().setPrettyPrinting().create().toJson(output);
    }
}
