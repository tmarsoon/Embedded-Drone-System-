package militarylogistics;

import javafx.animation.Timeline;

import java.util.List;

import javafx.animation.KeyFrame;
import javafx.util.Duration;
import javafx.geometry.Point3D;

public class TroopResponder {
    private List<TroopNodesFly> troops;

    public TroopResponder(List<TroopNodesFly> troops) {
        this.troops = troops;
    }

    public void respondToPlea(CityPlea plea) {
        TroopNodesFly closestTroop = findClosestTroopWithResource(plea);
        if (closestTroop != null) {
            moveToCity(closestTroop, plea.getCityLocation());
        }
    }

    private TroopNodesFly findClosestTroopWithResource(CityPlea plea) {
        TroopNodesFly closest = null;
        double minDistance = Double.MAX_VALUE;

        for (TroopNodesFly troop : troops) {
            if (troop.getResourceType().equals(plea.getRequiredResource())) {
                double distance = plea.getCityLocation().distance(troop.getTranslateX(), troop.getTranslateY(), troop.getTranslateZ());
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = troop;
                }
            }
        }
        return closest;
    }

    private void moveToCity(TroopNodesFly troop, Point3D cityLocation) {
        Timeline timeline = new Timeline(new KeyFrame(Duration.millis(100), e -> {
            Point3D direction = cityLocation.subtract(troop.getTranslateX(), troop.getTranslateY(), troop.getTranslateZ()).normalize();
            troop.setTranslateX(troop.getTranslateX() + direction.getX() * 5);
            troop.setTranslateY(troop.getTranslateY() + direction.getY() * 5);
            troop.setTranslateZ(troop.getTranslateZ() + direction.getZ() * 5);
        }));
        timeline.setCycleCount(Timeline.INDEFINITE);
        timeline.play();
    }
}