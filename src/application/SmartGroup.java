package application;

import javafx.scene.Group;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;

/*
 * In javaFX, a Group is aessentially a container that allows 3D objects to be manipulated
 * by using shapes, rotations, and translations; however, in Group, you cannot customly modify the translate and 
 * rotate methods. Utilizing SmartGroup (an extention of Group), allows one to provide
 * custom modifications. 
 */
public class SmartGroup extends Group {

	
	/*
	 * SmartGroup class acts as a flexible container that allows 
	 * you to rotate 3D objects, like turning the earth to see all sides.
	 * In my case, I can control the earth with my mouse.
	 * Rotate r is an object where each time I grab the earth with my mouse, 
	 * the xAxis method is called, creating a new Rotate r object (same
	 * for yAxis).
	*/
    Rotate r;
    Transform t = new Rotate();

    
    public void xAxis(int ang) {
        r = new Rotate(ang, Rotate.X_AXIS);
        /*t is the sum of movements, each new r rotation is added so essentially,
         * instead of consistently creating new methods and r objects each time,
         * that specific 3D movement you perform, may of already have been created, 
         * just calling a previous movement of the object. it adds new movements to an entire history of the objects
         * previous movements. 
         */
        t = t.createConcatenation(r);
        //clear, will clear all of the previous movements that existed in SmartGroup prior top your movements.
        this.getTransforms().clear();
        //all all movements to it's history
        this.getTransforms().addAll(t);
    }

    //same here but for yAxis
    public void yAxis(int ang) {
        r = new Rotate(ang, Rotate.Y_AXIS);
        t = t.createConcatenation(r);
        this.getTransforms().clear();
        this.getTransforms().addAll(t);
    }
}










