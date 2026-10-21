package militarylogistics;

import javafx.geometry.Point3D;

public class CityPlea {
    private Point3D cityLocation;
    private String requiredResource;

    public CityPlea(Point3D cityLocation, String requiredResource) {
        this.cityLocation = cityLocation;
        this.requiredResource = requiredResource;
    }

    public Point3D getCityLocation() {
        return cityLocation;
    }

    public String getRequiredResource() {
        return requiredResource;
    }
}