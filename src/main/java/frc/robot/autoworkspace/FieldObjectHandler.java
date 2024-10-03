package frc.robot.autoworkspace;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

//list of field objects
public class FieldObjectHandler {
    public static FieldObjectHandler instance = null;
    public static synchronized FieldObjectHandler getInstance() {
        if (instance == null) instance = new FieldObjectHandler();
        return instance;
    }

    List<FieldObject> fieldobjs;
    
    //constructor
    public FieldObjectHandler() {
        fieldobjs = new ArrayList<FieldObject>();
    }

    //constructor

    public FieldObjectHandler(FieldObject... fobj) {
        this();
        for (FieldObject f : fobj) {
            add(f);
        }
    }

    public void addPose2d(Pose2d pose, double r) {
        add(new FieldObject(pose, r));   
    }

    public void addPose2d(Pose2d pose, double w, double h) {
        add(new FieldObject(pose, w, h));
    }

    //add field object unless its already exists within
    public void add(FieldObject fobj) {
        if (fieldobjs.contains(fobj)) {return;}

        fieldobjs.add(fobj);
    }

    //removes field object unless it doesnt exist within
    public void remove(FieldObject fobj) {
        if (!fieldobjs.contains(fobj)) {return;}

        fieldobjs.remove(fieldobjs.indexOf(fobj));
    }

    //iterates through objects checking if point touches
    boolean pointIsTouchingAny(double x, double y) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.isTouching(x, y)) return true;
        }

        return false;
    }

    //iterates through objects checking if line crosses
    boolean lineIsTouchingAny(double x0, double y0, double x1, double y1) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.lineIsTouching(x0, y0, x1, y1)) return true;
        }

        return false;
    }
    
}
