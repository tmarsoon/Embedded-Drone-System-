package application;
import java.util.List;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;

/* 
 * This class represents the earth model in 3D.
 * The earth is displayed as a sphere and contains realistic textures. 
 */
public class Earth {
	//agianst the pane, im setting the radius of the earth to diameter of 600
    private static final double EARTH_RADIUS = 300;
    private Sphere earthSphere;
    private Group troopGroup;

    public Earth() {
        // Initialize the Earth sphere with a radius
        earthSphere = new Sphere(EARTH_RADIUS);
        troopGroup = new Group();

       //phong material will apply lighting to the earth to present a more round realistic model
        PhongMaterial earthCrust = new PhongMaterial();
        //earth d is a diffuse photo, meaning it emphases land masses and ocean in a photo
        earthCrust.setDiffuseMap(new Image(getClass().getResourceAsStream("earth-d.jpg"))); // Day texture
        //earth s is a specular map, meaing that it emphases that ceratin areas on the earth appear shinier than other during sun presence
        earthCrust.setSpecularMap(new Image(getClass().getResourceAsStream("earth-s.jpg"))); 
       
      //0 to 200 range, this sets the shininess of the earths crust that appears shinier to begin with
        earthCrust.setSpecularPower(140); 

      //setMaterial is a mathod in the Phong class that takes users altered configurations and applies it the surface of the photo
        earthSphere.setMaterial(earthCrust);
    }

    //getter to access the earth 
    public Sphere getEarthSphere() {
        return earthSphere;
    }
    

  //creating method to create the earth with the base nodes now 
    public Group createEarthWithBases(List<BaseNode> baseNodes) {
    	
    	/* 
    	 * in java fx I add earthsphere to a group and manipulate its children (or nodes) for data retrieval
    	 */
    	
    	Group earthGroup = new Group(earthSphere); 

        //iterates through the basenode list
    	//each basenode represents a military base, the label (or name), and the number of troops
        for (BaseNode baseNode : baseNodes) {
            earthGroup.getChildren().add(baseNode.getLabel());       
            baseNode.generateTroopBalls(baseNode.getTroopCount(), earthSphere);
            //adds all to the group
            troopGroup.getChildren().addAll(baseNode.troopNodes);
        }

        earthGroup.getChildren().add(troopGroup); 
        return earthGroup;
    }
}
	