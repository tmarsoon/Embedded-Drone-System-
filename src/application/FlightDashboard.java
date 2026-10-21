package application;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import integration.Hub;
import integration.HubCatalog;
import integration.Mission;
import integration.NativeSimulator;
import integration.SimulationRequest;
import integration.SimulationRun;
import integration.TelemetryPoint;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.collections.FXCollections;
import javafx.concurrent.Task;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SplitPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.util.Duration;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

/** JavaFX view over the authoritative C++ simulation output; no UI-side flight physics. */
public final class FlightDashboard extends BorderPane implements AutoCloseable {
    private static final Color[] COLORS = {Color.web("#38bdf8"), Color.web("#fb923c"),
        Color.web("#4ade80"), Color.web("#c084fc"), Color.web("#f472b6"), Color.web("#facc15")};
    private final NativeSimulator simulator;
    private final ExecutorService executor = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "native-flight-worker");
        thread.setDaemon(true);
        return thread;
    });
    private final ComboBox<Hub> hub = new ComboBox<>(FXCollections.observableArrayList(HubCatalog.all()));
    private final Spinner<Integer> drones = new Spinner<>(1, 8, 3);
    private final Spinner<Integer> deliveries = new Spinner<>(1, 100, 6);
    private final TextField seed = new TextField("42");
    private final TextField wind = new TextField("3.0");
    private final TextField gust = new TextField("0.8");
    private final TextField payload = new TextField("0.3");
    private final TextField gpsNoise = new TextField("0.25");
    private final Button runButton = new Button("Run flight");
    private final Button cancelButton = new Button("Cancel");
    private final Button playButton = new Button("Play");
    private final Label status = new Label("Choose a hub and run a physical flight scenario.");
    private final Label output = new Label("Results are saved locally for each run.");
    private final Label playbackTime = new Label("T + 0.0 s");
    private final Label windAtTime = new Label("Wind: —");
    private final Label completedMetric = new Label("—");
    private final Label acceptedMetric = new Label("—");
    private final Label energyMetric = new Label("—");
    private final Label separationMetric = new Label("—");
    private final Label errorMetric = new Label("—");
    private final Label noiseMetric = new Label("—");
    private final Canvas canvas = new Canvas(780, 390);
    private final Slider time = new Slider(0, 1, 0);
    private final ComboBox<Double> speed = new ComboBox<>(FXCollections.observableArrayList(1.0, 4.0, 8.0, 16.0, 32.0));
    private final CheckBox showEstimate = new CheckBox("Estimated position");
    private final TableView<LiveRow> liveTable = new TableView<>();
    private final LineChart<Number, Number> altitudeChart = chart("Altitude", "Metres");
    private final LineChart<Number, Number> batteryChart = chart("Battery", "Wh");
    private final Timeline playback;
    private final FlowPane parameterPane = new FlowPane(10, 8);
    private Task<SimulationRun> task;
    private SimulationRun current;
    private Map<Integer, List<TelemetryPoint>> byDrone = Map.of();
    private double minX = -180, maxX = 180, minY = -180, maxY = 180;
    private boolean playing;
    private boolean closed;

    public FlightDashboard() throws IOException {
        simulator = new NativeSimulator();
        setPadding(new Insets(14));
        setStyle("-fx-background-color:#0b1220;");
        hub.getSelectionModel().selectFirst();
        hub.setPrefWidth(225);
        drones.setEditable(false);
        deliveries.setEditable(false);
        drones.setPrefWidth(78);
        deliveries.setPrefWidth(84);
        for (TextField field : List.of(seed, wind, gust, payload, gpsNoise)) field.setPrefWidth(75);
        parameterPane.getChildren().addAll(field("Hub context", hub), field("Aircraft", drones),
            field("Deliveries", deliveries), field("Seed", seed), field("Wind m/s", wind),
            field("Gust σ m/s", gust), field("Parcel kg", payload), field("GPS σ m", gpsNoise));
        parameterPane.setPadding(new Insets(0, 0, 10, 0));
        runButton.setStyle("-fx-background-color:#0284c7;-fx-text-fill:white;-fx-font-weight:bold;-fx-padding:9 20;");
        cancelButton.setDisable(true);
        runButton.setOnAction(e -> startScenario(null, null));
        cancelButton.setOnAction(e -> cancel());
        status.setWrapText(true);
        status.setStyle("-fx-text-fill:#cbd5e1;-fx-font-size:12px;");
        HBox actions = new HBox(10, runButton, cancelButton, status);
        actions.setAlignment(Pos.CENTER_LEFT);
        HBox.setHgrow(status, Priority.ALWAYS);
        Label description = new Label("Local ENU flights · C++ dynamics, Kalman navigation, wind, parcel impact and battery reserves");
        description.setWrapText(true);
        description.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:12px;");
        VBox top = new VBox(5, parameterPane, actions, description, metrics());
        top.setPadding(new Insets(0, 0, 10, 0));
        setTop(top);

        StackPane drawing = new StackPane(canvas);
        drawing.setMinHeight(220);
        drawing.setPrefHeight(370);
        drawing.setStyle("-fx-background-color:#111d30;-fx-background-radius:8;");
        canvas.widthProperty().bind(drawing.widthProperty());
        canvas.heightProperty().bind(drawing.heightProperty());
        canvas.widthProperty().addListener((o, a, b) -> draw());
        canvas.heightProperty().addListener((o, a, b) -> draw());
        showEstimate.setSelected(true);
        showEstimate.selectedProperty().addListener((o, a, b) -> draw());
        speed.getSelectionModel().select(Double.valueOf(8));
        speed.setPrefWidth(80);
        speed.setTooltip(new Tooltip("Playback speed multiplier; computation is already complete."));
        time.setDisable(true);
        HBox.setHgrow(time, Priority.ALWAYS);
        playButton.setDisable(true);
        playButton.setOnAction(e -> {
            if (current == null) return;
            if (time.getValue() >= current.duration()) time.setValue(0);
            setPlaying(!playing);
        });
        time.valueProperty().addListener((o, a, b) -> updatePlayback());
        HBox transport = new HBox(10, playButton, time, playbackTime, new Label("Speed ×"), speed, showEstimate);
        transport.setAlignment(Pos.CENTER_LEFT);
        transport.setPadding(new Insets(8, 0, 0, 0));
        VBox flight = new VBox(0, drawing, transport, windAtTime);
        VBox.setVgrow(drawing, Priority.ALWAYS);
        windAtTime.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:11px;");
        liveTable.setColumnResizePolicy(TableView.CONSTRAINED_RESIZE_POLICY_FLEX_LAST_COLUMN);
        liveTable.setPlaceholder(new Label("Aircraft telemetry appears after a run."));
        liveTable.getColumns().addAll(column("Aircraft", LiveRow::identity), column("Phase", LiveRow::phase),
            column("Altitude", LiveRow::altitude), column("Battery", LiveRow::battery), column("Parcel", LiveRow::parcel));
        liveTable.setMinHeight(106);
        liveTable.setPrefHeight(106);
        HBox charts = new HBox(10, altitudeChart, batteryChart);
        HBox.setHgrow(altitudeChart, Priority.ALWAYS);
        HBox.setHgrow(batteryChart, Priority.ALWAYS);
        charts.setMinHeight(165);
        charts.setPrefHeight(205);
        VBox lower = new VBox(8, liveTable, charts);
        VBox.setVgrow(charts, Priority.ALWAYS);
        SplitPane split = new SplitPane(flight, lower);
        split.setOrientation(javafx.geometry.Orientation.VERTICAL);
        split.setDividerPositions(0.50);
        setCenter(split);
        output.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:11px;");
        output.setWrapText(true);
        output.setPadding(new Insets(9, 0, 0, 0));
        setBottom(output);
        playback = new Timeline(new KeyFrame(Duration.millis(40), e -> {
            if (current == null || !playing) return;
            time.setValue(Math.min(current.duration(), time.getValue() + 0.04 * speed.getValue()));
            if (time.getValue() >= current.duration()) setPlaying(false);
        }));
        playback.setCycleCount(Timeline.INDEFINITE);
        draw();
    }

    public ObjectProperty<Hub> selectedHubProperty() { return hub.valueProperty(); }
    public Path projectRoot() { return simulator.projectRoot(); }
    public SimulationRun currentRun() { return current; }
    public boolean hasRenderedTelemetry() {
        return current != null && !current.telemetry().isEmpty() && !altitudeChart.getData().isEmpty()
            && !liveTable.getItems().isEmpty() && canvas.getWidth() > 0 && canvas.getHeight() > 0;
    }

    /** Uses exactly the same native invocation/result/rendering path as the Run button. */
    public void startSmoke(Consumer<SimulationRun> success, Consumer<Throwable> failure) {
        drones.getValueFactory().setValue(3);
        deliveries.getValueFactory().setValue(6);
        seed.setText("42"); wind.setText("3"); gust.setText("0.8"); payload.setText("0.3"); gpsNoise.setText("0.25");
        startScenario(success, failure);
    }

    public void pauseAt(double seconds) {
        setPlaying(false);
        time.setValue(Math.max(0, Math.min(time.getMax(), seconds)));
    }

    private void startScenario(Consumer<SimulationRun> success, Consumer<Throwable> failure) {
        if (task != null && task.isRunning()) return;
        final SimulationRequest request;
        try {
            request = new SimulationRequest(hub.getValue().id(), drones.getValue(), deliveries.getValue(),
                Long.parseLong(seed.getText().trim()), number(wind), number(gust), number(payload), number(gpsNoise));
        } catch (RuntimeException error) {
            status.setText("Check the scenario values: " + error.getMessage());
            if (failure != null) failure.accept(error);
            return;
        }
        setPlaying(false);
        setBusy(true);
        status.setText("Computing the physical scenario in C++…");
        task = new Task<>() {
            @Override protected SimulationRun call() throws Exception {
                return simulator.run(request, java.time.Duration.ofMinutes(3));
            }
        };
        task.setOnSucceeded(e -> {
            setBusy(false);
            showRun(task.getValue());
            if (success != null) success.accept(current);
        });
        task.setOnFailed(e -> {
            setBusy(false);
            Throwable error = task.getException();
            status.setText("Run failed: " + message(error));
            if (failure != null) failure.accept(error);
        });
        task.setOnCancelled(e -> {
            setBusy(false);
            status.setText("Run cancelled. No completed result was loaded.");
            if (failure != null) failure.accept(new InterruptedException("Run cancelled"));
        });
        executor.execute(task);
    }

    private static String message(Throwable error) {
        return error == null ? "unknown error" : (error.getMessage() == null ? error.getClass().getSimpleName() : error.getMessage());
    }

    private static double number(TextField field) {
        double result = Double.parseDouble(field.getText().trim());
        if (!Double.isFinite(result)) throw new IllegalArgumentException("values must be finite");
        return result;
    }

    private void setBusy(boolean busy) {
        runButton.setDisable(busy);
        cancelButton.setDisable(!busy);
        parameterPane.setDisable(busy);
    }

    private void cancel() {
        simulator.cancel();
        if (task != null) task.cancel(true);
    }

    private void showRun(SimulationRun result) {
        current = result;
        byDrone = new LinkedHashMap<>(result.byDrone());
        JsonObject m = result.metrics();
        completedMetric.setText(integer(m, "completed") + " / " + integer(m, "requested"));
        acceptedMetric.setText(integer(m, "within_radius") + " in radius");
        energyMetric.setText(format(numeric(m, "energy_wh"), 2) + " Wh");
        separationMetric.setText(format(numeric(m, "minimum_separation_m"), 2) + " m");
        noiseMetric.setText(format(numeric(m, "max_noise_db"), 1) + " dB");
        List<Double> errors = result.missions().stream().filter(Mission::delivered)
            .map(Mission::impactError).filter(java.util.Objects::nonNull).sorted().toList();
        errorMetric.setText(errors.isEmpty() ? "n/a" : format(percentile(errors, 0.95), 2) + " m");
        boolean timeout = m.has("timed_out") && m.get("timed_out").getAsBoolean();
        int rejected = integer(m, "rejected");
        int conflicts = integer(m, "separation_events");
        status.setText(String.format(Locale.ROOT,
            "%s · %.1f s simulated · %d rejected · %d separation events",
            timeout ? "Time limit reached" : "Scenario ready", result.duration(), rejected, conflicts));
        output.setText("Saved: " + result.directory() + "  ·  Physical metres are local to " + result.hub().name());
        calculateBounds();
        populateCharts();
        time.setMax(Math.max(0.1, result.duration()));
        time.setValue(0);
        time.setDisable(result.telemetry().isEmpty());
        playButton.setDisable(result.telemetry().isEmpty());
        updatePlayback();
        if (!result.telemetry().isEmpty()) setPlaying(true);
    }

    private void calculateBounds() {
        minX = minY = -30; maxX = maxY = 30;
        for (TelemetryPoint p : current.telemetry()) {
            minX = Math.min(minX, p.x()); maxX = Math.max(maxX, p.x());
            minY = Math.min(minY, p.y()); maxY = Math.max(maxY, p.y());
        }
        for (Mission m : current.missions()) {
            minX = Math.min(minX, m.destinationX()); maxX = Math.max(maxX, m.destinationX());
            minY = Math.min(minY, m.destinationY()); maxY = Math.max(maxY, m.destinationY());
            if (m.delivered() && m.impactX() != null && m.impactY() != null) {
                minX = Math.min(minX, m.impactX()); maxX = Math.max(maxX, m.impactX());
                minY = Math.min(minY, m.impactY()); maxY = Math.max(maxY, m.impactY());
            }
        }
        double margin = Math.max(15, Math.max(maxX-minX, maxY-minY)*0.09);
        minX-=margin; maxX+=margin; minY-=margin; maxY+=margin;
    }

    private void populateCharts() {
        altitudeChart.getData().clear();
        batteryChart.getData().clear();
        for (var entry : byDrone.entrySet()) {
            XYChart.Series<Number,Number> alt = new XYChart.Series<>();
            XYChart.Series<Number,Number> bat = new XYChart.Series<>();
            alt.setName("Drone " + entry.getKey()); bat.setName(alt.getName());
            List<TelemetryPoint> points = entry.getValue();
            int step = Math.max(1, points.size()/350);
            for (int i=0; i<points.size(); i+=step) {
                TelemetryPoint p = points.get(i);
                alt.getData().add(new XYChart.Data<>(p.time(),p.z()));
                bat.getData().add(new XYChart.Data<>(p.time(),p.batteryWh()));
            }
            altitudeChart.getData().add(alt); batteryChart.getData().add(bat);
        }
    }

    private void updatePlayback() {
        playbackTime.setText(String.format(Locale.ROOT,"T + %.1f s", time.getValue()));
        List<LiveRow> rows = new ArrayList<>();
        for (var entry : byDrone.entrySet()) {
            TelemetryPoint p = at(entry.getValue(), time.getValue());
            if (p != null) rows.add(new LiveRow("Drone " + p.droneId(),p.phase(),format(p.z(),1)+" m",
                format(p.batteryWh(),1)+" Wh",p.payloadAttached()?"attached":"released"));
        }
        liveTable.setItems(FXCollections.observableArrayList(rows));
        if (!byDrone.isEmpty()) {
            TelemetryPoint p = byDrone.values().stream().map(points -> at(points,time.getValue()))
                .filter(java.util.Objects::nonNull).findFirst().orElse(null);
            if (p != null) windAtTime.setText(String.format(Locale.ROOT,
                "Sampled local wind: east %.2f · north %.2f · up %.2f m/s   |   Playback of saved C++ telemetry",
                p.windX(),p.windY(),p.windZ()));
        }
        draw();
    }

    private static TelemetryPoint at(List<TelemetryPoint> points, double t) {
        if (points.isEmpty() || t < points.get(0).time()) return null;
        int low=0, high=points.size();
        while (low<high) {
            int middle=(low+high)>>>1;
            if (points.get(middle).time()<=t) low=middle+1; else high=middle;
        }
        return points.get(Math.max(0,low-1));
    }

    private void draw() {
        GraphicsContext g=canvas.getGraphicsContext2D();
        double width=canvas.getWidth(), height=canvas.getHeight();
        g.setFill(Color.web("#111d30")); g.fillRect(0,0,width,height);
        if (width<50 || height<50) return;
        double scale=Math.min((width-85)/(maxX-minX),(height-65)/(maxY-minY));
        double originX=(width-scale*(maxX-minX))/2-minX*scale;
        double originY=(height-scale*(maxY-minY))/2+maxY*scale;
        double grid=niceGrid(Math.max(maxX-minX,maxY-minY)/8);
        g.setStroke(Color.web("#26354b")); g.setLineWidth(1); g.setFill(Color.web("#8494ad"));
        for (double x=Math.ceil(minX/grid)*grid;x<=maxX;x+=grid) {
            double px=originX+x*scale; g.strokeLine(px,22,px,height-25);
            g.fillText(format(x,0),px+3,height-9);
        }
        for (double y=Math.ceil(minY/grid)*grid;y<=maxY;y+=grid) {
            double py=originY-y*scale; g.strokeLine(42,py,width-18,py);
            g.fillText(format(y,0),7,py-2);
        }
        g.setFill(Color.web("#cbd5e1"));
        g.fillText("North ↑  (metres)",12,16); g.fillText("East →",Math.max(70,width-75),height-9);
        g.setFill(Color.web("#f8fafc")); g.fillRect(originX-5,originY-5,10,10); g.fillText("Hub",originX+9,originY-8);
        if (current==null) {
            g.setFill(Color.web("#94a3b8"));
            g.fillText("Run a scenario to display computed aircraft paths and parcel impacts.",Math.max(48,width/2-215),height/2+35);
            return;
        }
        for (Mission m:current.missions()) {
            double x=originX+m.destinationX()*scale,y=originY-m.destinationY()*scale;
            g.setStroke(Color.web("#e2e8f0")); g.setLineWidth(1.5);
            g.strokeLine(x-4,y-4,x+4,y+4); g.strokeLine(x-4,y+4,x+4,y-4);
            g.setFill(Color.web("#cbd5e1")); g.fillText("Site "+m.jobId(),x+7,y-5);
            if (m.delivered() && m.impactX()!=null && m.impactY()!=null && m.impactAt()<=time.getValue()) {
                g.setStroke(m.withinRadius()?Color.web("#4ade80"):Color.web("#fb7185"));
                g.strokeOval(originX+m.impactX()*scale-4,originY-m.impactY()*scale-4,8,8);
            }
        }
        for (var entry:byDrone.entrySet()) {
            Color color=COLORS[Math.floorMod(entry.getKey(),COLORS.length)];
            List<TelemetryPoint> points=entry.getValue();
            g.setStroke(new Color(color.getRed(),color.getGreen(),color.getBlue(),0.16));
            g.setLineWidth(1); drawPath(g,points,Double.POSITIVE_INFINITY,originX,originY,scale);
            g.setStroke(color); g.setLineWidth(1.8);
            drawPath(g,points,time.getValue(),originX,originY,scale);
            TelemetryPoint p=at(points,time.getValue());
            if (p==null) continue;
            double x=originX+p.x()*scale,y=originY-p.y()*scale;
            g.setFill(color); g.fillOval(x-4.5,y-4.5,9,9);
            g.fillText("D"+p.droneId()+" · "+format(p.z(),0)+"m",x+7,y+13);
            if (showEstimate.isSelected()) {
                double ex=originX+p.estimatedX()*scale, ey=originY-p.estimatedY()*scale;
                g.setStroke(Color.web("#f8fafc")); g.setLineWidth(1);
                g.strokeOval(ex-6,ey-6,12,12);
            }
        }
    }

    private static void drawPath(GraphicsContext g,List<TelemetryPoint> points,double cutoff,
                                 double originX,double originY,double scale) {
        boolean started=false;
        g.beginPath();
        for (int i=0;i<points.size();i++) {
            TelemetryPoint p=points.get(i);
            if (p.time()>cutoff) break;
            if (i%3!=0 && i+1<points.size() && points.get(i+1).time()<=cutoff) continue;
            double x=originX+p.x()*scale,y=originY-p.y()*scale;
            if (!started) {g.moveTo(x,y);started=true;} else g.lineTo(x,y);
        }
        g.stroke();
    }

    private static double niceGrid(double wanted) {
        double power=Math.pow(10,Math.floor(Math.log10(Math.max(1,wanted))));
        double scaled=wanted/power;
        return (scaled<=1?1:scaled<=2?2:scaled<=5?5:10)*power;
    }

    private void setPlaying(boolean value) {
        playing=value;
        playButton.setText(value?"Pause":"Play");
        if (value) playback.play(); else playback.pause();
    }

    private GridPane metrics() {
        GridPane grid=new GridPane(); grid.setHgap(10);grid.setVgap(6);grid.setPadding(new Insets(8,0,0,0));
        Label[] values={completedMetric,acceptedMetric,energyMetric,separationMetric,errorMetric,noiseMetric};
        String[] titles={"Returned / requested","Accepted parcels","Fleet energy","Minimum separation","Impact error p95","Modeled peak sound"};
        for(int i=0;i<values.length;i++) {
            ColumnConstraints col=new ColumnConstraints();col.setPercentWidth(100.0/values.length);grid.getColumnConstraints().add(col);
            values[i].setStyle("-fx-text-fill:#f8fafc;-fx-font-size:20px;-fx-font-weight:bold;");
            Label title=new Label(titles[i]);title.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:11px;");
            VBox card=new VBox(3,title,values[i]);card.setPadding(new Insets(9));
            card.setStyle("-fx-background-color:#17243a;-fx-background-radius:7;");grid.add(card,i,0);
        }
        return grid;
    }

    private static VBox field(String label,javafx.scene.Node control) {
        Label title=new Label(label);title.setStyle("-fx-text-fill:#94a3b8;-fx-font-size:11px;");
        return new VBox(4,title,control);
    }

    private static LineChart<Number,Number> chart(String title,String unit) {
        NumberAxis x=new NumberAxis(),y=new NumberAxis();x.setLabel("Time (s)");y.setLabel(unit);
        LineChart<Number,Number> chart=new LineChart<>(x,y);chart.setTitle(title);
        chart.setAnimated(false);chart.setCreateSymbols(false);chart.setMinWidth(250);chart.setPrefHeight(205);
        chart.setLegendVisible(true);return chart;
    }

    private static TableColumn<LiveRow,String> column(String title,java.util.function.Function<LiveRow,String> getter) {
        TableColumn<LiveRow,String> column=new TableColumn<>(title);
        column.setCellValueFactory(cell->new ReadOnlyStringWrapper(getter.apply(cell.getValue())));
        return column;
    }

    private static int integer(JsonObject object,String key) {return object.get(key).getAsInt();}
    private static double numeric(JsonObject object,String key) {
        JsonElement value=object.get(key);return value==null||value.isJsonNull()?Double.NaN:value.getAsDouble();
    }
    private static String format(double value,int decimals) {
        return Double.isFinite(value)?String.format(Locale.ROOT,"%."+decimals+"f",value):"n/a";
    }
    private static double percentile(List<Double> values,double p) {
        double index=p*(values.size()-1);int low=(int)Math.floor(index),high=(int)Math.ceil(index);
        return values.get(low)+(index-low)*(values.get(high)-values.get(low));
    }

    private record LiveRow(String identity,String phase,String altitude,String battery,String parcel) {}

    @Override public void close() {
        if (closed) return;
        closed=true;playback.stop();cancel();executor.shutdownNow();
    }
}
