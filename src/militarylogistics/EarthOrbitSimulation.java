package militarylogistics;

import javafx.animation.AnimationTimer;
import javafx.geometry.Point3D;


import java.util.List;
import java.util.Random;

import application.Logger;
import application.LogisticsVisualization2;
import application.Troop;
public class EarthOrbitSimulation {
//fixed center of the Earth
private Point3D earthCenter;
//random generator for simulations
private Random random = new Random();
//single Troop instance for takeoff
private Troop trooper;
//list of troops in orbit
private List<TroopNodesFly> troops;

public EarthOrbitSimulation(Troop trooper, List<TroopNodesFly> troops, Point3D earthCenter) {
    //initialize trooper
    this.trooper = trooper;
    //initialize list of troops
    this.troops = troops;
    //set fixed center of the Earth
    this.earthCenter = earthCenter;
}

public void startOrbitSimulation(LogisticsVisualization2 logisticsVis) {
    //display troops and base visuals
    logisticsVis.display();
    //initialize the troop takeoff sequence
    initializeTakeoffSequence(logisticsVis);

    //start AnimationTimer to manage troop movement
    new AnimationTimer() {
        //track last update time
        private long lastUpdate = 0;

        @Override
        public void handle(long now) {
            //initialize last update time
            if (lastUpdate == 0) {
                lastUpdate = now;
            }
            //calculate time delta
            double deltaTime = (now - lastUpdate) / 1_000_000_000.0;
            //update last update time
            lastUpdate = now;

            //update positions for each TroopNodesFly instance
            for (TroopNodesFly troop : troops) {
                //get current angle of troop
                double currentAngle = troop.getCurrentAngle();
                //define speed factor
                double speedFactor = 0.1;
                //calculate new angle
                double newAngle = currentAngle + troop.getOrbitSpeed() * deltaTime * speedFactor;
                //update troop's current angle
                troop.setCurrentAngle(newAngle);

                //calculate new position and update troop's location
                Point3D newPosition = calculateOrbitPosition(earthCenter, troop.getOrbitRadius(), newAngle);
                //set x position
                troop.setTranslateX(newPosition.getX());
                //set y position
                troop.setTranslateY(newPosition.getY());
                //set z position
                troop.setTranslateZ(newPosition.getZ());

                //update combustion monitoring
                updateCombustionMonitoring(troop);
                //update position tracking
                updatePositionTracking(troop);

                //log new position
                System.out.println(trooper.getName() + "position: " + newPosition);
                //log to file
                Logger.log(trooper.getName() + "position: " + newPosition);
            }
        }
    }.start(); //start animation timer
}

private void initializeTakeoffSequence(LogisticsVisualization2 logisticsVis) {
    //get list of added troops
    List<Troop> addedTroops = logisticsVis.getAddedTroops();

    //iterate over each added troop
    for (Troop trooper : addedTroops) {
        //log takeoff
        System.out.println(trooper.getName() + " from the " + trooper.getBranch() + " taking off.");
        //log takeoff to file
        Logger.log(trooper.getName() + " from the " + trooper.getBranch() + " taking off.");

        try {
            //simulate delay for takeoff
            Thread.sleep(150);
        } catch (InterruptedException e) {
            //handle interruption
            e.printStackTrace();
        }
    }
    //log final statement
    System.out.println("All troops are now circumventing the globe");
    //log final statement to file
    Logger.log("All troops are now circumventing the globe");
}

private Point3D calculateOrbitPosition(Point3D center, double radius, double angle) {
    //set inclination angle
    double inclination = Math.toRadians(45);
    //define random factor for variation
    double randomFactor = 0.05;

    //calculate primary orbit coordinates based on Earth's center
    double x = center.getX() + radius * Math.cos(angle) * Math.sin(inclination);
    double y = center.getY() + radius * Math.sin(angle) * Math.sin(inclination);
    double z = center.getZ() + radius * Math.cos(inclination);

    //add small random offsets to create dispersed orbit
    x += (Math.random() - 0.5) * radius * randomFactor;
    y += (Math.random() - 0.5) * radius * randomFactor;
    z += (Math.random() - 0.5) * radius * randomFactor;

    //return new position
    return new Point3D(x, y, z);
}

private void updateCombustionMonitoring(TroopNodesFly troop) {
    //update and get estimated temperature
    double estimatedTemperature = troop.updateCombustionTemperature();
    //log temperature
    System.out.println("Estimated Combustion Temperature for " + trooper.getName() + ": " + estimatedTemperature + "°F");
    //log temperature to file
    Logger.log("Estimated Combustion Temperature for " + trooper.getName() + ": " + estimatedTemperature + "°F");
   
    //log warning if temperature exceeds safe limits
    if (estimatedTemperature > 5500) {
        System.out.println("Warning: Temperature exceeding safe limits for " + troop.getName() + "! Risk of explosion.");
        Logger.log("Warning: Temperature exceeding safe limits for " + troop.getName() + "! Risk of explosion.");
    }
}

private void updatePositionTracking(TroopNodesFly troop) {
    //simulate accelerometer and GPS data
    Point3D accelerometerData = new Point3D(
            random.nextDouble() * 500,
            random.nextDouble() * 500,
            random.nextDouble() * 500
    );
    Point3D gpsData = new Point3D(
            random.nextDouble() * 500,
            random.nextDouble() * 500,
            random.nextDouble() * 500
    );

    //80% chance GPS is strong
    boolean gpsSignalStrong = random.nextDouble() > 0.2;

    //update position based on GPS and accelerometer with Kalman filtering
    Point3D estimatedPosition = troop.updatePosition(accelerometerData, gpsData, gpsSignalStrong);
    //log position
    System.out.println("Estimated Position for " + troop.getName() + ": " + estimatedPosition + "\n");
    //log position to file
    Logger.log("Estimated Position for " + troop.getName() + ": " + estimatedPosition + "\n");
    
    //log GPS interruption if signal is weak
    if (!gpsSignalStrong) {
        System.out.println("GPS interruption for " + troop.getName() + ": relying on accelerometer data.");
        Logger.log("GPS interruption for " + troop.getName() + ": relying on accelerometer data.");
    }
}

public void simulateCityPleaAndAssignResponse() {
    //generate random city plea
    CityPlea cityPlea = generateRandomCityPlea();
    //log city plea
    System.out.println("City at " + cityPlea.getCityLocation() + " requests " + cityPlea.getRequiredResource());

    //select nearest troop to respond
    TroopNodesFly assignedTroop = selectNearestTroop(cityPlea);
    if (assignedTroop != null) {
        //log assignment of troop to respond
        System.out.println("Assigning " + assignedTroop.getName() + " to respond to the city plea.");
        //initiate response to plea
        respondToPlea(assignedTroop, cityPlea);
    } else {
        //log if no troop is available
        System.out.println("No troops with the required resources available to respond to city plea.");
    }
}

private CityPlea generateRandomCityPlea() {
    //generate random location
    Point3D location = new Point3D(random.nextInt(300) - 150, random.nextInt(300) - 150, 0);
    //define possible resources for plea
    String[] resources = {"water", "food", "ammo", "vaccines"};
    //return new city plea with a randomly selected resource
    return new CityPlea(location, resources[random.nextInt(resources.length)]);
}

private TroopNodesFly selectNearestTroop(CityPlea plea) {
    //initialize nearest troop
    TroopNodesFly nearest = null;
    //set initial max distance
    double minDistance = Double.MAX_VALUE;

    //iterate over each troop
    for (TroopNodesFly troop : troops) {
        //check if troop has required resource
        if (troop.getResourceType().contains(plea.getRequiredResource())) {
            //calculate distance to city plea location
            double distance = troop.getTranslateX() - plea.getCityLocation().getX() +
                              troop.getTranslateY() - plea.getCityLocation().getY() +
                              troop.getTranslateZ() - plea.getCityLocation().getZ();
            //update nearest troop if distance is shorter
            if (distance < minDistance) {
                minDistance = distance;
                nearest = troop;
            }
        }
    }
    //return nearest troop
    return nearest;
}

    private void respondToPlea(TroopNodesFly troop, CityPlea plea) {
        double distanceToCity = troop.getTranslateX() - plea.getCityLocation().getX() +
                                troop.getTranslateY() - plea.getCityLocation().getY() +
                                troop.getTranslateZ() - plea.getCityLocation().getZ();

        boolean explosion = random.nextDouble() < 0.1;

        while (distanceToCity > 10) {
            double estimatedTemp = troop.updateCombustionTemperature();
            Point3D estimatedPos = troop.updatePosition(new Point3D(250, 250, 250), plea.getCityLocation(), true);

            System.out.println(troop.getName() + " - Temperature: " + estimatedTemp + "°F");
            System.out.println("Position: (" + estimatedPos.getX() + ", " + estimatedPos.getY() + ", " + estimatedPos.getZ() + ")");

            if (explosion) {
                System.out.println("Engine explosion! " + troop.getName() + " is ejecting.");
                TroopNodesFly backupTroop = selectNearestTroop(plea);
                if (backupTroop != null && backupTroop != troop) {
                    System.out.println("Reassigning to " + backupTroop.getName());
                    troop = backupTroop;
                }
                break;
            }

            distanceToCity = estimatedPos.getX() - plea.getCityLocation().getX() +
                             estimatedPos.getY() - plea.getCityLocation().getY() +
                             estimatedPos.getZ() - plea.getCityLocation().getZ();

            try { Thread.sleep(400); } catch (InterruptedException e) { e.printStackTrace(); }
        }

        System.out.println(troop.getName() + " has reached the city.");
    }
}
