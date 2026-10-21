package application;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

/** Append-only logging for the retained legacy demonstration. */
public final class Logger {
    private Logger() { }

    /**
     * Historical uploaded logs are never opened or truncated. A launcher may set
     * drone.project.root (ustranscomm.root is a compatibility fallback), or
     * override ustranscomm.logDir for a particular run.
     * Resolve properties per call so tests and embedded applications can isolate
     * output without class-loading side effects.
     */
    public static synchronized void log(String message) {
        String configured = System.getProperty("ustranscomm.logDir", "").trim();
        Path directory = configured.isEmpty()
                ? Path.of(System.getProperty("drone.project.root",
                        System.getProperty("ustranscomm.root", System.getProperty("user.dir"))))
                        .resolve("runs").resolve("logs")
                : Path.of(configured);
        try {
            Files.createDirectories(directory);
            Files.writeString(directory.resolve("simulation.log"),
                    String.valueOf(message) + System.lineSeparator(), StandardCharsets.UTF_8,
                    StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (IOException | SecurityException error) {
            System.err.println("Unable to append legacy simulation log: " + error.getMessage());
        }
    }
}
