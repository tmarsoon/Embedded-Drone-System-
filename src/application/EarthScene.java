package application;

import javafx.animation.AnimationTimer;
import javafx.geometry.Point2D;
import javafx.geometry.Point3D;
import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Text;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import militarylogistics.TroopNodesFly;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;


public class EarthScene {

    private List<BaseNode> baseNodes; // Store base nodes here
    private Group earthGroup; // Group containing the Earth sphere and base nodes
    private List<TroopNodesFly> troopBalls;

    public EarthScene() {
        this.baseNodes = new ArrayList<>();
        this.troopBalls = new ArrayList<>();
        this.earthGroup = new Group();
    }

    public Group createEarthAndBaseScene(Earth earth, List<BaseNode> baseNodes, Camera camera) {
    	 Sphere earthSphere = earth.getEarthSphere(); // Retrieve the Earth sphere
    	    earthGroup.getChildren().add(earthSphere); // Add the Earth sphere to the group

    	    for (BaseNode base : baseNodes) {
    	        this.baseNodes.add(base); // Store the base node
    	        // Calculate the 3D position of the base node based on its latitude and longitude
    	        Point3D basePosition = latLonTo3D(earthSphere.getRadius(), base.getLatitude(), base.getLongitude());
    	        // Set the position of the base sphere and its label
    	        base.getBaseSphere().setTranslateX(basePosition.getX());
    	        base.getBaseSphere().setTranslateY(basePosition.getY());
    	        base.getBaseSphere().setTranslateZ(basePosition.getZ());
    	        base.getLabel().setTranslateX(basePosition.getX());
    	        base.getLabel().setTranslateY(basePosition.getY() - 10); // Adjust label position above the base
    	        base.getLabel().setTranslateZ(basePosition.getZ());

    	        // Add the base sphere and label to the Earth group
    	        earthGroup.getChildren().addAll(base.getBaseSphere(), base.getLabel());

    	        // Generate troop balls for the base and add them to the troop group
    	        base.generateTroopBalls(base.getTroopCount(), earthSphere);
    	        troopBalls.addAll(base.getTroopBalls()); // Collect troop nodes
    	    }

    	    earthGroup.getChildren().addAll(troopBalls); // Add all troop nodes to the Earth group

    	    // Use AnimationTimer to orient labels towards the camera
    	    new AnimationTimer() {
    	        @Override
    	        public void handle(long now) {
    	            for (BaseNode base : baseNodes) {
    	                orientLabelTowardsCamera(base.getLabel(), camera); // Adjust label orientation
    	            }
    	        }
    	    }.start();

    	    return earthGroup; // Return the completed Earth group
    	}

    // Adjusts label rotation to face the camera
    private void orientLabelTowardsCamera(Text label, Camera camera) {
        Point3D cameraPosition = camera.localToScene(Point3D.ZERO);
        Point3D direction = cameraPosition.subtract(
            label.getTranslateX(), label.getTranslateY(), label.getTranslateZ()
        );
        double angleY = Math.toDegrees(Math.atan2(direction.getX(), direction.getZ()));
        double angleX = Math.toDegrees(Math.atan2(direction.getY(), direction.getZ()));
        label.getTransforms().setAll(
            new Rotate(-angleY, Rotate.Y_AXIS),
            new Rotate(angleX, Rotate.X_AXIS)
        );
    }

    // Helper method to convert latitude and longitude to 3D Cartesian coordinates
    private Point3D latLonTo3D(double radius, double latitude, double longitude) {
        double latRad = Math.toRadians(latitude);
        double lonRad = Math.toRadians(longitude);
        double x = radius * Math.cos(latRad) * Math.sin(lonRad);
        double y = -radius * Math.sin(latRad);
        double z = radius * Math.cos(latRad) * Math.cos(lonRad);
        return new Point3D(x, y, z);
    }

    // Method to return all base nodes as a list
    public List<BaseNode> getBases() {
        return baseNodes;
    }
}