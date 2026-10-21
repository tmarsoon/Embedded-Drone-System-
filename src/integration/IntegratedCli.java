package integration;

import java.nio.file.Path;
import java.time.Duration;

/** Headless entry point using exactly the same native bridge as the JavaFX dashboard. */
public final class IntegratedCli {
    private IntegratedCli() {}
    public static void main(String[] args) {
        try {
            String hub="offutt";
            int drones=3, deliveries=6;
            long seed=42;
            double wind=3,gust=0.8,payload=0.3,gps=0.25;
            Path output=null;
            for (int index=0;index<args.length;++index) {
                String option=args[index];
                if (option.equals("--help")) {
                    System.out.println("IntegratedCli [--hub ID] [--drones N] [--deliveries N] [--seed N] [--wind MPS] [--gust MPS] [--payload KG] [--gps-noise M] [--output NEW_DIRECTORY]\nHub IDs: offutt, north-island, gregg-adams, marine-district\nFull model parameters and Monte Carlo studies remain available through native/build/drone_sim --help.");
                    return;
                }
                if (++index>=args.length) throw new IllegalArgumentException("Missing value for " + option);
                String value=args[index];
                switch (option) {
                    case "--hub" -> hub=value;
                    case "--drones" -> drones=Integer.parseInt(value);
                    case "--deliveries" -> deliveries=Integer.parseInt(value);
                    case "--seed" -> seed=Long.parseLong(value);
                    case "--wind" -> wind=Double.parseDouble(value);
                    case "--gust" -> gust=Double.parseDouble(value);
                    case "--payload" -> payload=Double.parseDouble(value);
                    case "--gps-noise" -> gps=Double.parseDouble(value);
                    case "--output" -> output=Path.of(value);
                    default -> throw new IllegalArgumentException("Unknown option: " + option);
                }
            }
            SimulationRequest request=new SimulationRequest(hub,drones,deliveries,seed,wind,gust,payload,gps);
            try (NativeSimulator simulator=new NativeSimulator()) {
                SimulationRun run=output==null?simulator.run(request,Duration.ofMinutes(2))
                        :simulator.run(request,output,Duration.ofMinutes(2));
                System.out.println(NativeSimulator.georeferencedReport(run));
                System.err.println("Integrated run files: " + run.directory());
            }
        } catch (Exception exception) {
            if (exception instanceof InterruptedException) Thread.currentThread().interrupt();
            System.err.println("IntegratedCli: " + exception.getMessage());
            System.exit(1);
        }
    }
}
