package militarylogistics;

import javafx.geometry.Point3D;
import javafx.scene.Group;
import java.util.ArrayList;
import java.util.List;

public class TroopDeployment {
    private List<TroopNodesFly> deployedTroops = new ArrayList<>();

    public List<TroopNodesFly> deployTroops(Group earthGroup, int numberOfTroops, Point3D orbitCenter) {
        for (int i = 0; i < numberOfTroops; i++) {
            String resourceType = assignRandomResource();
            TroopNodesFly troop = new TroopNodesFly(resourceType, resourceType, resourceType, orbitCenter, 300 + Math.random() * 50, 0.001 + Math.random() * 0.003);
            deployedTroops.add(troop);
            earthGroup.getChildren().add(troop);
        }
        return deployedTroops;
    }

    private String assignRandomResource() {
        String[] resources = {"Food", "Water", "Medical", "Ammunition"};
        return resources[(int) (Math.random() * resources.length)];
    }

    public List<TroopNodesFly> getDeployedTroops() {
        return deployedTroops;
    }
}