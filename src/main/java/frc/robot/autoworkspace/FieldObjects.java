package frc.robot.autoworkspace;
import java.util.ArrayList;

public class FieldObjects {
    ArrayList<FieldObject> fieldobjs;
    
    FieldObjects() {}

    void add(FieldObject fobj) {
        if (fieldobjs.contains(fobj)) {return;}

        fieldobjs.add(fobj);
    }

    void remove(FieldObject fobj) {
        if (!fieldobjs.contains(fobj)) {return;}

        fieldobjs.remove(fieldobjs.indexOf(fobj));
    }

    boolean pointIsTouchingAny(double x, double y) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.isTouching(x, y)) return true;
        }

        return false;
    }

    boolean lineIsTouchingAny(double x0, double y0, double x1, double y1) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.lineIsTouching(x0, y0, x1, y1)) return true;
        }

        return false;
    }
    
}
