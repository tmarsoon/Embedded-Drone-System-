package application;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

public class Galaxy extends Application {

	  private static final float WIDTH = 1400;
	  private static final float HEIGHT = 1000;

	  @Override
	  public void start(Stage primaryStage) {
	    Group root = new Group();

	    // Add galaxy background image
	    ImageView galaxyImageView = prepareImageView();
	    root.getChildren().add(galaxyImageView);

	    Scene scene = new Scene(root, WIDTH, HEIGHT, true);
	    scene.setFill(Color.SILVER);

	    primaryStage.setTitle("Galaxy Background Example");
	    primaryStage.setScene(scene);
	    primaryStage.show();
	  }

	  // Prepare the ImageView to display the galaxy image as a background
	  public ImageView prepareImageView() {
	    Image image = new Image(getClass().getResourceAsStream("galaxy.jpg")); // Ensure the correct path
	    ImageView imageView = new ImageView(image);

	    // Set ImageView size to cover the entire scene
	    imageView.setFitWidth(WIDTH); // Match the width of the scene
	    imageView.setFitHeight(HEIGHT); // Match the height of the scene
	    imageView.setPreserveRatio(false); // Allow stretching to cover the full background

	    return imageView;
	  }

	  public static void main(String[] args) {
	    launch(args); // Launch JavaFX application
	  }

	  // Method to provide the Galaxy ImageView for layering in Main
	  public ImageView getGalaxyBackground() {
	    return prepareImageView();
	  }
	}