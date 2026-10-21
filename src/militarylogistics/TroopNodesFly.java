package militarylogistics;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.geometry.Point3D;

public class TroopNodesFly extends Sphere {
    private String name;
    private String branch;
    private String resourceType;
    private double orbitRadius;
    private double orbitSpeed;
    private double currentAngle = 0;

    private KalmanFilter combustionKalmanFilter; // Kalman filter for combustion temperature
    private KalmanFilter gpsKalmanFilter; // Kalman filter for GPS/accelerometer tracking

    public TroopNodesFly(String name, String branch, String resourceType, Point3D orbitCenter, double orbitRadius, double orbitSpeed) {
        super(3); // Set radius of each troop node
        this.name = name;
        this.branch = branch;
        this.resourceType = resourceType;
        this.orbitRadius = orbitRadius;
        this.orbitSpeed = orbitSpeed;

        // Initialize Kalman filters with starting estimates
        this.combustionKalmanFilter = new KalmanFilter(3000, 200, 500); // Starting values for combustion
        this.gpsKalmanFilter = new KalmanFilter(0, 1, 0.5); // Starting values for GPS tracking

        // Set visual appearance of the troop node
        setMaterial(createMaterial());
    }

    // Create and set the visual material for troop node
    private PhongMaterial createMaterial() {
        PhongMaterial material = new PhongMaterial();
        material.setDiffuseColor(Color.RED.deriveColor(1, 1, 1, 0.6)); // Semi-transparent red
        material.setSpecularColor(Color.WHITE); // Reflective shine
        material.setSpecularPower(32); // Sharp highlights
        return material;
    }

    // Getters for troop properties
    public String getName() {
        return name;
    }

    public String getBranch() {
        return branch;
    }

    public String getResourceType() {
        return resourceType;
    }

    public double getOrbitSpeed() {
        return orbitSpeed;
    }

    public double getOrbitRadius() {
        return orbitRadius;
    }

    // Getter and setter for current angle (used in orbit calculations)
    public double getCurrentAngle() {
        return currentAngle;
    }

    public void setCurrentAngle(double currentAngle) {
        this.currentAngle = currentAngle;
    }

    // Simulate combustion temperature readings with Kalman filtering
    public double updateCombustionTemperature() {
        double rawTemperature = 3000 + Math.random() * 500; // Simulated baseline temp
        double coolingInfluence = Math.random() * 50; // Cooling influence from cylinders
        double adjustedTemp = rawTemperature - coolingInfluence;

        // Filtered temperature estimate
        double estimatedTemp = combustionKalmanFilter.update(adjustedTemp);
        return estimatedTemp;
    }

    // Simulate position updates with Kalman filtering, using accelerometer data if GPS signal is weak
    public Point3D updatePosition(Point3D accelerometerData, Point3D gpsData, boolean gpsSignalStrong) {
        double x, y, z;
        if (gpsSignalStrong) {
            // GPS position update
            x = gpsKalmanFilter.update(gpsData.getX());
            y = gpsKalmanFilter.update(gpsData.getY());
            z = gpsKalmanFilter.update(gpsData.getZ());
        } else {
            // Use accelerometer data due to GPS interruption
            x = gpsKalmanFilter.update(accelerometerData.getX());
            y = gpsKalmanFilter.update(accelerometerData.getY());
            z = gpsKalmanFilter.update(accelerometerData.getZ());
            System.out.println("GPS interruption for " + name + ": relying on accelerometer data.");
        }
        Point3D estimatedPosition = new Point3D(x, y, z);
        return estimatedPosition;
    }
}
