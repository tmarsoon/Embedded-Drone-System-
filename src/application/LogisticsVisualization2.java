
package application;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Platform;
import javafx.geometry.Point2D;
import javafx.geometry.Point3D;
import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Text;
import javafx.scene.transform.Transform;
import javafx.util.Duration;
import java.util.List;
import java.util.Random;
import java.io.IOException;
import java.util.ArrayList;


public class LogisticsVisualization2 {
    private Pane canvas = new Pane();
    private List<Circle> troops = new ArrayList<>();
    private List<Text> troopLabels = new ArrayList<>();
    private List<BaseNode> bases = new ArrayList<>();
    private Random random = new Random(42L);
   
    private String csvFilePath = "medal_of_honor.csv"; 
    private List<Troop> addedTroops = new ArrayList<>(); 
    private int currentTroopIndex = 0;
    private Camera camera;
    private Earth earth;
    private Scene scene;  // <-- Scene reference added
    private List<Troop> troopNames;
    private Troop currentTroop;  
    private boolean initialized;
    private Timeline updateTimeline;
    
    public LogisticsVisualization2(Scene scene) {
        this.scene = scene;
    }

    public Pane display() {
        if (initialized) {
            return canvas;
        }
        try {
        	 CSVReader csvReader = new CSVReader();
             troopNames = csvReader.readTroopNames(csvFilePath);

             if (troopNames == null || troopNames.isEmpty()) {
                 System.out.println("No troop names loaded from CSV file.");
                 return canvas;
             }

             // Initialize camera for scene
             camera = new PerspectiveCamera(true);
             camera.setNearClip(0.1);
             camera.setFarClip(10000.0);
             camera.translateZProperty().set(-1000);

             // Initialize Earth and EarthScene, which includes Earth and base nodes
             earth = new Earth();
             EarthScene earthScene = new EarthScene();

             // Define troop counts for each base
             int offuttTroops = 10;
             int navalStationTroops = 15;
             int fortGreggTroops = 20;
             int firstMarineCorpsDistrict = 15;

             // Create a list of BaseNode instances for each base
             List<BaseNode> baseNodes = new ArrayList<>();
             baseNodes.add(new BaseNode("Offutt AFB.", offuttTroops, 41.118, -95.912, earth.getEarthSphere()));
             baseNodes.add(new BaseNode("Naval Air Station North Island.", navalStationTroops, 32.699, -117.215, earth.getEarthSphere()));
             baseNodes.add(new BaseNode("Fort Gregg-Adams.", fortGreggTroops, 37.239, -77.338, earth.getEarthSphere()));
             baseNodes.add(new BaseNode("1st Marine Corps District.", firstMarineCorpsDistrict, 40.7, -73.6, earth.getEarthSphere()));
             // Add Earth with bases to canvas
             canvas.getChildren().add(earthScene.createEarthAndBaseScene(earth, baseNodes, camera));

             // Populate the bases list with base nodes from EarthScene
             bases.addAll(earthScene.getBases()); // Ensure EarthScene has a getBases() method returning base nodes

             // Timeline to add troops at intervals
             updateTimeline = new Timeline(new KeyFrame(Duration.seconds(0.1), event -> {
                 addNextTroop(); // Add a troop to the base
             }));
             updateTimeline.setCycleCount(troopNames.size());
             initialized = true;
             updateTimeline.play();

         } catch (IOException e) {
             System.out.println("Failed to load troop names from CSV.");
             e.printStackTrace();
         }

         return canvas;
     }
    
    

    // Create a troop at a random position around the base
    private Circle createTroop(double x, double y) {
        return new Circle(x, y, 5, Color.RED);
    }
    public List<BaseNode> getBaseNodes() {
        return bases; // Return the initialized base nodes
    }
   
   
    //add the next troop to a random base
    private void addNextTroop() {
    	 if (currentTroopIndex < troopNames.size()) {
    		
    	        // Select a random troop from the list
    	       //Troop troopName = troopNames.get(random.nextInt(troopNames.size()));
    		 currentTroop = troopNames.get(currentTroopIndex);
    	        // Determine the base based on troop's branch
    	        BaseNode assignedBase = null;
    	        switch (currentTroop.getBranch()) {
    	            case "U.S. Air Force":
    	                assignedBase = findBaseByName("Offutt AFB.");
    	                break;
    	            case "U.S. Navy":
    	                assignedBase = findBaseByName("Naval Air Station North Island.");
    	                break;
    	            case "U.S. Army":
    	                assignedBase = findBaseByName("Fort Gregg-Adams.");
    	                break;
    	            case "U.S. Marine Corps":
    	                assignedBase = findBaseByName("1st Marine Corps District.");
    	                break;
    	            default:
    	          
    	                break;
    	        }

    	      
    	        if (assignedBase != null) {
    	            
    	            double x = assignedBase.getCenterX() + random.nextDouble() * 40 - 20;
    	            double y = assignedBase.getCenterY() + random.nextDouble() * 40 - 20;

    	            Circle troop = createTroop(x, y);
    	            Text label = new Text(currentTroop.getName() + " (" + currentTroop.getBranch() + ")");
    	            label.setTranslateX(x + 5);
    	            label.setTranslateY(y - 5);

    	            troops.add(troop);
    	            troopLabels.add(label);
    	            canvas.getChildren().addAll(troop, label);
    	           addedTroops.add(currentTroop); 
    	          System.out.println("Troop added: " + currentTroop + " assigned to " + assignedBase.getUserData());
    	           Logger.log("Troop added: " + currentTroop.getName() + " assigned to " + assignedBase.getUserData());
    	          
    	        } else {
    	            System.out.println("Could not assign " + currentTroop.getName() + " from " +  currentTroop.getBranch() + " to a base.");
    	            Logger.log("Could not assign " + currentTroop.getName() + " from " +  currentTroop.getBranch() + " to a base.");
    	        }
    	       //System.out.println(currentTroop + " from " + currentTroop + " taking off.");
    	       // Logger.log(currentTroop + " from " + currentTroop + " taking off.");
    	        currentTroopIndex++;
    	    }
    	
    	}

    	//method to find a base by its name
    	private BaseNode findBaseByName(String baseName) {
    	    for (BaseNode base : bases) {
    	        if (base.getUserData().equals(baseName)) {
    	            return base;
    	        }
    	    }
    	    return null;
    	}
    	public List<Troop> getTroopNames() {
            return troopNames; 
        }
    	public Troop getCurrentTroop() {
            return currentTroop;
        }
    	public List<Troop> getAddedTroops() {
    	    return addedTroops;
    	}

    /** Stop the retained demonstration when its scene is no longer displayed. */
    public void stop() {
        if (updateTimeline != null) {
            updateTimeline.stop();
        }
    }
    	
}
