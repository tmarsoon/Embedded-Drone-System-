package application;

import java.util.ArrayList;

import java.util.List;
import java.util.Random;
import java.util.Objects;

import javafx.geometry.Point3D;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Text;
import militarylogistics.TroopNodesFly;

/*Highlight....

The BaseNode class is responsible for setting up the bases of the military camps on the 
globe in a 3D space. This class is also responsible for finding the geographic locations
on the earth. This class also generates the troop instances that will deploy
to circumvent the globe. 
 
*/ 
public class BaseNode {
	private static final double NODE_RADIUS = 5;
    private Sphere baseSphere;
    private Text label;
    
    /*Please note----
     * 
     * the latitude and longitude coordinates are being based off of the
     * the actual earth, not the pane. For this reason, the actual longitude 
     * latitude coordinates of the 3 bases is off compared to its true location.
     * The reason for this is because JavaFX introduce a multivariable approach for creating 3D objects and 
     * the variable of depth Z, alters the position. 
     */
   //base latitude
    private double latitude;  
    //bases longitude
    private double longitude; 
    private Object nameOfBase; 
    //list of troops being deployed to the data
    List<TroopNodesFly> troopNodes; 
    public List<String> resourcesHeld;
    private int troopCount; 
    private int generatedCount = -1;
    private double generatedEarthRadius = Double.NaN;


    public BaseNode(String baseName, int troopCount, double latitude, double longitude, Sphere earthSphere) {
        //setting variables
    	this.latitude = latitude;
        this.longitude = longitude;
        this.troopCount = troopCount;
        this.troopNodes = new ArrayList<>();
        //each troop is going to have these four items on board for when Plea of Help is called
        resourcesHeld = List.of("food", "water", "ammo", "vaccines");
        //creating a sphere for the base node
        baseSphere = new Sphere(NODE_RADIUS);
        /*utilized PhoneMaterial which is a computer graphics technique to alter the light reflection of a 3D object
        this give a more realistic image because it smoothes the objects surfaces because the pixels on ur screen
        can alter the smoothness of the object by appearing more rigid.*/
        baseSphere.setMaterial(new PhongMaterial(Color.RED));
        
        //holds the basesname
        this.nameOfBase = baseName;

        // Position the base node on the Earth model
        nodesPositionOnSurface(baseSphere, earthSphere, latitude, longitude);

        // Create a label to display the base name and troop count
        label = new Text(baseName + ": " + troopCount);
        label.setFill(Color.WHITE);
        label.setTranslateX(baseSphere.getTranslateX());
        label.setTranslateY(baseSphere.getTranslateY() - 20); // Position label above node
        label.setTranslateZ(baseSphere.getTranslateZ());
    }

    public void generateTroopBalls(int numTroops, Sphere earthSphere) {
        Objects.requireNonNull(earthSphere, "earthSphere");
        if (numTroops < 0) {
            throw new IllegalArgumentException("troop count must not be negative");
        }
        if (generatedCount == numTroops && generatedEarthRadius == earthSphere.getRadius()) {
            return; // Repeated display requests reuse the existing scene nodes.
        }
        troopNodes.clear();
        generatedCount = numTroops;
        generatedEarthRadius = earthSphere.getRadius();
        Random generation = new Random(Objects.hash(nameOfBase, latitude, longitude, numTroops));
    	List<String> branches = List.of("U.S. Marine Corps", "U.S. Air Force", "U.S. Army", "U.S. Navy");

        for (int i = 0; i < numTroops; i++) {
            // Randomize orbit radius and speed for each troop node
            double orbitRadius = earthSphere.getRadius() + 50 + generation.nextDouble() * 50;
            double orbitSpeed = 0.5 + generation.nextDouble() * 2;

            // Randomize initial position vector (orbit randomization)
            double vx = (generation.nextDouble() - 0.5) * 2;
            double vy = (generation.nextDouble() - 0.5) * 2;
            double vz = (generation.nextDouble() - 0.5) * 2;
            Point3D orbitRandom = new Point3D(vx, vy, vz);

            // Generate random name and branch, and cycle through resources list
            String name = "Troop " + (i + 1); // Generate a simple name like "Troop 1", "Troop 2", etc.
            String branch = branches.get(i % branches.size()); // Cycle through branches list
            String resourceType = resourcesHeld.get(i % resourcesHeld.size()); // Cycle through resources list

            // Create a TroopNodesFly instance with the new arguments
            TroopNodesFly troopNode = new TroopNodesFly(name, branch, resourceType, orbitRandom, orbitRadius, orbitSpeed);

            // Add the troop node to the list
            troopNodes.add(troopNode);
        }
    }
    /**
     * this method positions a node on the earth's surface based on lat. and long.
     * 
     * @param node        sphere node to position
     * @param earthSphere sphere representing the earth
     * @param latitude    latitude 
     * @param longitude   longitude 
     */
    private void nodesPositionOnSurface(Sphere node, Sphere earthSphere, double latitude, double longitude) {
        
    	double radius = earthSphere.getRadius();
        Point3D position = latLonTo3D(radius, latitude, longitude);
        //then coordinates ion the main method will apply to the node with translate, which moves the node
        //along its axis' to the specified location
        node.setTranslateX(position.getX());
        node.setTranslateY(position.getY());
        node.setTranslateZ(position.getZ());
    }

    /**
     * converts latitude and longitude to 3D cartesian coordinates.
     *  
     * In java, Point3D is a class that is used for defining coordinates in a 3 dimensional space
     * @param radius   the radius of earth
     * @param latitude the latitude in degrees
     * @param longitude the longitude in degrees
     * @return A Point3D object for cart. coordinates
     */
    private Point3D latLonTo3D(double radius, double latitude, double longitude) {
        double latRad = Math.toRadians(latitude);
        double lonRad = Math.toRadians(longitude);

        double x = radius * Math.cos(latRad) * Math.sin(lonRad);
        double y = -radius * Math.sin(latRad);
        double z = radius * Math.cos(latRad) * Math.cos(lonRad);

        return new Point3D(x, y, z);
    }
    
    //retrieves the troopNodes for each base
    public List<TroopNodesFly> getTroopBalls() {
        return troopNodes;
    }

    //gets the base sphere 
    public Sphere getBaseSphere() {
        return baseSphere;
    }
    //gets the label
    public Text getLabel() {
        return label;
    }

    //returning the x coordinate of the centering position of the basenode
    public double getCenterX() {
        return baseSphere.getTranslateX();
    }

    //returning the y coordinate of the centering position of the basenode
    public double getCenterY() {
        return baseSphere.getTranslateY();
    }

    //gets the users data
    public Object getUserData() {
        return nameOfBase;
    }
    
    //getter for number of troops deployed to each base node
    public int getTroopCount() {
        return troopCount;
    }

    //setting users data
    public void setUserData(Object userData) {
        this.nameOfBase = userData;
    }

    //getter for latitude
    public double getLatitude() {
        return latitude;
    }
    
    //getter for longitude
    public double getLongitude() {
        return longitude;
    }
    
    
}
