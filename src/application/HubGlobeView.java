package application;

import integration.Hub;
import integration.HubCatalog;
import javafx.application.ConditionalFeature;
import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.geometry.Point3D;
import javafx.geometry.Pos;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Tooltip;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.Consumer;

/** Geographic context for local metre-scale missions, rather than orbital motion. */
public final class HubGlobeView extends BorderPane {
    private static final Color ACTIVE = Color.web("#fbbf24");
    private static final Color INACTIVE = Color.web("#38bdf8");
    private static final double EARTH_RADIUS = 300.0;
    private final Label selectedLabel = new Label();
    private final Label coordinates = new Label();
    private final Map<String, Sphere> markers = new LinkedHashMap<>();
    private final Map<String, Circle> flatMarkers = new LinkedHashMap<>();
    private final Map<String, Button> hubButtons = new LinkedHashMap<>();
    private final Rotate longitudeRotation = new Rotate(0, Rotate.Y_AXIS);
    private final Rotate latitudeRotation = new Rotate(0, Rotate.X_AXIS);
    private Consumer<Hub> selectionListener = hub -> { };
    private Hub selected;
    private double pointerX;
    private double pointerY;

    public HubGlobeView() {
        setStyle("-fx-background-color: #0b1220;");
        setPadding(new Insets(20));
        Label title = new Label("Geographic hub context");
        title.setStyle("-fx-text-fill: #f1f5f9; -fx-font-size: 23px; -fx-font-weight: bold;");
        Label description = new Label("Choose a hub, then run and inspect its local flights in the Flight lab.");
        description.setWrapText(true);
        description.setStyle("-fx-text-fill: #94a3b8; -fx-font-size: 13px;");
        VBox heading = new VBox(6, title, description);
        heading.setPadding(new Insets(0, 0, 16, 0));
        setTop(heading);

        StackPane viewport = new StackPane();
        viewport.setMinSize(280, 300);
        viewport.setStyle("-fx-background-color: #020617; -fx-background-radius: 12;");
        ImageView galaxy = new ImageView(new Image(getClass().getResourceAsStream("galaxy.jpg")));
        galaxy.setPreserveRatio(false);
        galaxy.setOpacity(0.35);
        galaxy.fitWidthProperty().bind(viewport.widthProperty());
        galaxy.fitHeightProperty().bind(viewport.heightProperty());
        viewport.getChildren().add(galaxy);

        if (Platform.isSupported(ConditionalFeature.SCENE3D)) {
            addGlobe(viewport);
        } else {
            addMapFallback(viewport);
        }
        Label interaction = new Label("Drag to rotate · Scroll to zoom");
        interaction.setMouseTransparent(true);
        interaction.setStyle("-fx-text-fill: #cbd5e1; -fx-background-color: rgba(2,6,23,0.8); "
                + "-fx-padding: 7 10; -fx-background-radius: 6;");
        if (!Platform.isSupported(ConditionalFeature.SCENE3D)) {
            interaction.setText("Geographic map · 3D acceleration unavailable in this renderer");
        }
        StackPane.setAlignment(interaction, Pos.BOTTOM_LEFT);
        StackPane.setMargin(interaction, new Insets(12));
        viewport.getChildren().add(interaction);
        setCenter(viewport);

        VBox selector = new VBox(10);
        selector.setPadding(new Insets(12, 0, 0, 18));
        selector.setPrefWidth(270);
        Label selectorHeading = new Label("HUB LOCATIONS");
        selectorHeading.setStyle("-fx-text-fill: #94a3b8; -fx-font-weight: bold; -fx-font-size: 11px;");
        selector.getChildren().add(selectorHeading);
        for (Hub hub : HubCatalog.all()) {
            Button button = new Button(hub.name());
            button.setMaxWidth(Double.MAX_VALUE);
            button.setWrapText(true);
            button.setAlignment(Pos.CENTER_LEFT);
            button.setOnAction(event -> {
                setHub(hub);
                selectionListener.accept(hub);
            });
            hubButtons.put(hub.id(), button);
            selector.getChildren().add(button);
        }
        Label context = new Label("Base locations provide the geographic origin. Flight distances, altitude, "
                + "wind and parcel drops are computed in a local east/north/up coordinate frame.");
        context.setWrapText(true);
        context.setStyle("-fx-text-fill: #94a3b8; -fx-font-size: 12px; -fx-line-spacing: 4;");
        selector.getChildren().add(context);
        setRight(selector);

        selectedLabel.setStyle("-fx-text-fill: #fbbf24; -fx-font-weight: bold; -fx-font-size: 15px;");
        coordinates.setStyle("-fx-text-fill: #94a3b8;");
        Label scale = new Label("Globe for geographic context. Metre-scale mission trajectories are shown separately at readable scale.");
        scale.setWrapText(true);
        scale.setStyle("-fx-text-fill: #94a3b8; -fx-font-size: 12px;");
        HBox details = new HBox(18, selectedLabel, coordinates);
        details.setAlignment(Pos.CENTER_LEFT);
        HBox.setHgrow(selectedLabel, Priority.ALWAYS);
        VBox footer = new VBox(7, details, scale);
        footer.setPadding(new Insets(14, 0, 0, 0));
        setBottom(footer);
        if (!HubCatalog.all().isEmpty()) setHub(HubCatalog.all().get(0));
    }

    private void addGlobe(StackPane viewport) {
        Sphere earth = new Earth().getEarthSphere();
        Group geography = new Group(earth);
        for (Hub hub : HubCatalog.all()) {
            Sphere marker = new Sphere(5.0, 20);
            Point3D point = onSphere(EARTH_RADIUS + 3.0, hub.latitude(), hub.longitude());
            marker.setTranslateX(point.getX());
            marker.setTranslateY(point.getY());
            marker.setTranslateZ(point.getZ());
            marker.setMaterial(new PhongMaterial(INACTIVE));
            Tooltip.install(marker, new Tooltip(hub.name()));
            marker.setOnMouseClicked(event -> {
                setHub(hub);
                selectionListener.accept(hub);
                event.consume();
            });
            markers.put(hub.id(), marker);
            geography.getChildren().add(marker);
        }
        Group longitudeGroup = new Group(geography);
        longitudeGroup.getTransforms().add(longitudeRotation);
        Group latitudeGroup = new Group(longitudeGroup);
        latitudeGroup.getTransforms().add(latitudeRotation);
        javafx.scene.PointLight light = new javafx.scene.PointLight(Color.WHITE);
        light.setTranslateX(-700);
        light.setTranslateY(-500);
        light.setTranslateZ(-1000);
        Group root = new Group(latitudeGroup, new AmbientLight(Color.web("#a4b2c7")), light);
        SubScene scene = new SubScene(root, 720, 540, true, SceneAntialiasing.BALANCED);
        scene.setFill(Color.TRANSPARENT);
        PerspectiveCamera camera = new PerspectiveCamera(true);
        camera.setNearClip(0.1);
        camera.setFarClip(10000.0);
        camera.setTranslateZ(-1300.0);
        scene.setCamera(camera);
        scene.widthProperty().bind(viewport.widthProperty());
        scene.heightProperty().bind(viewport.heightProperty());
        scene.setOnMousePressed(event -> {
            pointerX = event.getSceneX();
            pointerY = event.getSceneY();
        });
        scene.setOnMouseDragged(event -> {
            longitudeRotation.setAngle(longitudeRotation.getAngle() + (event.getSceneX()-pointerX)*0.35);
            latitudeRotation.setAngle(clamp(latitudeRotation.getAngle()-(event.getSceneY()-pointerY)*0.35, -85, 85));
            pointerX = event.getSceneX();
            pointerY = event.getSceneY();
        });
        scene.setOnScroll(event -> {
            camera.setTranslateZ(clamp(camera.getTranslateZ() + event.getDeltaY()*2.0, -2800, -720));
            event.consume();
        });
        viewport.getChildren().add(scene);
    }

    private void addMapFallback(StackPane viewport) {
        Pane map = new Pane();
        ImageView earth = new ImageView(new Image(getClass().getResourceAsStream("earth-d.jpg")));
        earth.setPreserveRatio(false);
        earth.fitWidthProperty().bind(map.widthProperty());
        earth.fitHeightProperty().bind(map.heightProperty());
        earth.setOpacity(0.85);
        map.getChildren().add(earth);
        for (Hub hub : HubCatalog.all()) {
            Circle marker = new Circle(5, INACTIVE);
            marker.centerXProperty().bind(map.widthProperty().multiply((hub.longitude()+180.0)/360.0));
            marker.centerYProperty().bind(map.heightProperty().multiply((90.0-hub.latitude())/180.0));
            marker.setStroke(Color.web("#020617"));
            marker.setStrokeWidth(1.5);
            Tooltip.install(marker, new Tooltip(hub.name()));
            marker.setOnMouseClicked(event -> {
                setHub(hub);
                selectionListener.accept(hub);
            });
            flatMarkers.put(hub.id(), marker);
            map.getChildren().add(marker);
        }
        StackPane wrapper = new StackPane(map);
        wrapper.setMaxHeight(440);
        viewport.getChildren().add(wrapper);
    }

    public void setHub(Hub hub) {
        if (hub == null) return;
        selected = hub;
        selectedLabel.setText(hub.name());
        coordinates.setText(String.format(Locale.ROOT, "%.4f° %s  ·  %.4f° %s",
                Math.abs(hub.latitude()), hub.latitude() >= 0 ? "N" : "S",
                Math.abs(hub.longitude()), hub.longitude() >= 0 ? "E" : "W"));
        latitudeRotation.setAngle(hub.latitude());
        longitudeRotation.setAngle(180.0-hub.longitude());
        markers.forEach((id, marker) -> {
            boolean active = id.equals(hub.id());
            marker.setRadius(active ? 7.0 : 5.0);
            marker.setMaterial(new PhongMaterial(active ? ACTIVE : INACTIVE));
        });
        flatMarkers.forEach((id, marker) -> {
            marker.setRadius(id.equals(hub.id()) ? 7.0 : 5.0);
            marker.setFill(id.equals(hub.id()) ? ACTIVE : INACTIVE);
        });
        hubButtons.forEach((id, button) -> button.setStyle(id.equals(hub.id())
                ? "-fx-background-color: #17334a; -fx-text-fill: #fbbf24; -fx-padding: 12; -fx-border-color: #38bdf8; -fx-border-radius: 5; -fx-background-radius: 5;"
                : "-fx-background-color: #172133; -fx-text-fill: #cbd5e1; -fx-padding: 12; -fx-border-color: #26364d; -fx-border-radius: 5; -fx-background-radius: 5;"));
    }

    public Hub getHub() { return selected; }
    public void setOnHubSelected(Consumer<Hub> listener) {
        selectionListener = listener == null ? hub -> { } : listener;
    }

    private static Point3D onSphere(double radius, double latitude, double longitude) {
        double lat = Math.toRadians(latitude), lon = Math.toRadians(longitude);
        return new Point3D(radius*Math.cos(lat)*Math.sin(lon), -radius*Math.sin(lat),
                radius*Math.cos(lat)*Math.cos(lon));
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
