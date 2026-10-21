package application;

import integration.SimulationRun;
import javafx.animation.PauseTransition;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.embed.swing.SwingFXUtils;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextArea;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.util.Duration;

import javax.imageio.ImageIO;
import java.awt.Desktop;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/** Desktop entry point for the uploaded JavaFX interface and integrated C++ engine. */
public final class Main extends Application {
    private FlightDashboard dashboard;
    private PauseTransition smokeWatchdog;

    @Override public void start(Stage stage) {
        boolean smoke = getParameters().getRaw().contains("--smoke");
        try {
            dashboard = new FlightDashboard();
            HubGlobeView globe = new HubGlobeView();
            globe.setHub(dashboard.selectedHubProperty().get());
            dashboard.selectedHubProperty().addListener((o,a,b)->globe.setHub(b));
            globe.setOnHubSelected(hub->dashboard.selectedHubProperty().set(hub));

            TabPane tabs = new TabPane(tab("Flight lab",dashboard),tab("Geographic hubs",globe),
                tab("Historical roster",new RosterPane(dashboard.projectRoot())),
                tab("Research & model",researchPane(dashboard.projectRoot())));
            tabs.setTabClosingPolicy(TabPane.TabClosingPolicy.UNAVAILABLE);
            Label title = new Label("HUB & SPOKE  /  FLIGHT LAB");
            title.setStyle("-fx-font-size:21px;-fx-font-weight:bold;-fx-text-fill:#f8fafc;");
            Label subtitle = new Label("JavaFX mission workspace · C++ physical simulation · Reproducible research");
            subtitle.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:12px;");
            Button openRun = new Button("Open run folder");
            Label openStatus = new Label();openStatus.setStyle("-fx-text-fill:#fbbf24;-fx-font-size:11px;");
            openRun.setOnAction(e->{
                Path directory=dashboard.currentRun()==null?dashboard.projectRoot().resolve("runs"):dashboard.currentRun().directory();
                try {openFolder(directory);openStatus.setText("");}
                catch(Exception error){openStatus.setText(error.getMessage());}
            });
            VBox heading=new VBox(4,title,subtitle);
            HBox header=new HBox(18,heading,openStatus,openRun);HBox.setHgrow(heading,Priority.ALWAYS);
            header.setPadding(new Insets(16,20,14,20));
            BorderPane root=new BorderPane(tabs);root.setTop(header);
            root.setStyle("-fx-base:#17243a;-fx-background:#0b1220;-fx-control-inner-background:#111d30;"
                +"-fx-text-base-color:#e2e8f0;-fx-text-background-color:#e2e8f0;-fx-background-color:#0b1220;"
                +"-fx-font-family:'SansSerif';-fx-font-size:12px;");
            Scene scene=new Scene(root,1360,960);
            stage.setTitle("USTRANSCOMM · Hub-and-spoke Flight Lab");
            stage.setMinWidth(1040);stage.setMinHeight(760);stage.setScene(scene);stage.show();
            stage.setOnCloseRequest(e->close());
            if(smoke) startSmoke(scene);
        } catch(Exception error) {
            if(smoke){smokeFailed(error);return;}
            TextArea text=new TextArea("The application could not start.\n\n"+error+"\n\nLaunch from the project folder, or set -Ddrone.project.root to that folder. See README.md for build commands.");
            text.setEditable(false);text.setWrapText(true);
            stage.setScene(new Scene(text,850,360));stage.setTitle("Flight Lab — startup error");stage.show();
        }
    }

    private void startSmoke(Scene scene) {
        smokeWatchdog=new PauseTransition(Duration.seconds(60));
        smokeWatchdog.setOnFinished(e->smokeFailed(new IOException("UI smoke test exceeded 60 seconds")));
        smokeWatchdog.play();
        Platform.runLater(()->dashboard.startSmoke(result->{
            try {
                require(result.metrics().get("requested").getAsInt()==6,"smoke requested count");
                require(result.metrics().get("completed").getAsInt()==6,"native missions did not all return");
                require(result.metrics().get("within_radius").getAsInt()==6,"native parcel acceptance count");
                require(result.metrics().get("separation_events").getAsInt()==0,"native separation events");
                require(!result.telemetry().isEmpty(),"native telemetry is empty");
                dashboard.pauseAt(result.duration()*0.55);
                require(dashboard.hasRenderedTelemetry(),"dashboard did not render telemetry and charts");
                PauseTransition pulse=new PauseTransition(Duration.millis(350));
                pulse.setOnFinished(e->{
                    try {
                        String name=getParameters().getRaw().stream().filter(a->a.startsWith("--smoke-output="))
                            .map(a->a.substring("--smoke-output=".length())).findFirst().orElse("runs/ui-smoke.png");
                        Path path=Path.of(name).toAbsolutePath().normalize();
                        Files.createDirectories(path.getParent());
                        WritableImage image=scene.snapshot(null);
                        if(!ImageIO.write(SwingFXUtils.fromFXImage(image,null),"png",path.toFile()))
                            throw new IOException("PNG image writer unavailable");
                        smokeWatchdog.stop();
                        System.out.println("UI_SMOKE_PASS completed=6 accepted=6 telemetry="+result.telemetry().size()
                            +" screenshot="+path+" run="+result.directory());
                        Platform.exit();
                    } catch(Exception error){smokeFailed(error);}
                });
                pulse.play();
            } catch(Exception error){smokeFailed(error);}
        },Main::smokeFailed));
    }

    private static void require(boolean condition,String message) {
        if(!condition)throw new IllegalStateException(message);
    }

    private static void smokeFailed(Throwable error) {
        System.err.println("UI_SMOKE_FAIL "+error);
        error.printStackTrace(System.err);
        System.exit(2);
    }

    private static Tab tab(String title,javafx.scene.Node node) {return new Tab(title,node);}

    private static BorderPane researchPane(Path root) {
        BorderPane pane=new BorderPane();pane.setPadding(new Insets(18));
        Label title=new Label("Research questions, measured answers and assumptions");
        title.setStyle("-fx-font-size:21px;-fx-font-weight:bold;");
        Label note=new Label("The native model is numerically tested, not calibrated to a particular aircraft or Argonne flight data. Existing research answers describe the bundled model and seeds.");
        note.setWrapText(true);
        Label command=new Label("Reproduce all scenarios:  python3 native/tools/research.py --binary native/build/drone_sim --output runs/research --runs 30");
        command.setWrapText(true);command.setStyle("-fx-font-family:Monospaced;-fx-text-fill:#7dd3fc;");
        VBox top=new VBox(9,title,note,command);top.setPadding(new Insets(0,0,12,0));pane.setTop(top);
        TextArea answers=new TextArea();answers.setEditable(false);answers.setWrapText(true);
        answers.setStyle("-fx-font-family:Monospaced;-fx-font-size:12px;");
        try {answers.setText(Files.readString(root.resolve("native/docs/RESEARCH_ANSWERS.md"),StandardCharsets.UTF_8));}
        catch(IOException error){answers.setText("The research report could not be loaded: "+error.getMessage()+"\n\nSee native/README.md and native/docs/MODEL.md. The flight simulator remains available.");}
        pane.setCenter(answers);return pane;
    }

    private static void openFolder(Path directory) throws IOException {
        Files.createDirectories(directory);
        if(!Desktop.isDesktopSupported()||!Desktop.getDesktop().isSupported(Desktop.Action.OPEN))
            throw new IOException("Open this folder in your file explorer: "+directory);
        Desktop.getDesktop().open(directory.toFile());
    }

    private void close() {
        if(smokeWatchdog!=null)smokeWatchdog.stop();
        if(dashboard!=null)dashboard.close();
    }

    @Override public void stop(){close();}
    public static void main(String[]args){launch(args);}
}
