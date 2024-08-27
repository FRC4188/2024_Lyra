package frc.robot.autoworkspace;
import java.util.ArrayList;

//list of field objects
public class FieldObjects {
    ArrayList<FieldObject> fieldobjs;
    
    //constructor
    FieldObjects() {
        fieldobjs = new ArrayList<FieldObject>();
    }

    //add field object unless its already exists within
    void add(FieldObject fobj) {
        if (fieldobjs.contains(fobj)) {return;}

        fieldobjs.add(fobj);
    }

    //removes field object unless it doesnt exist within
    void remove(FieldObject fobj) {
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
