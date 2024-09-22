package frc.robot.autoworkspace;
import edu.wpi.first.math.geometry.Pose2d;

//field object class, goals for path or be obstacles
public class FieldObject {
    double x, y, w = -1, h = -1, r = -1;
    boolean obstacle = true;

    //point constructor
    public FieldObject(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public FieldObject(Pose2d pose) {
        this(pose.getX(), pose.getY());
    }

    //circle constructor
    public FieldObject(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public FieldObject(Pose2d pose, double r) {
        this(pose.getX(), pose.getY(), r);
    }

    //rect constructor
    public FieldObject(double x, double y, double w, double h) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
    }

    public FieldObject(Pose2d pose, double w,double h) {
        this(pose.getX(), pose.getY(), w, h);
    }

    //point is touching inside shape or point
    public boolean isTouching(double x, double y) {
        if (!obstacle) return false;

        if (r != -1) {
        	return (x - this.x) * (x - this.x) + (y - this.y) * (y - this.y) <= r * r;
        }
        else if (w != -1 && h != -1) {
        	return (this.x - 0.5 * w <= x && x <= this.x + 0.5 * w && this.y - 0.5 * h <= y && y <= this.y + 0.5 * h);
        }
        else {
        	return (this.x == x && this.y == y);
        }
    }

    //lines is touching through shape or point
    public boolean lineIsTouching(double x0, double y0, double x1, double y1) {
        if (!obstacle) return false;

        if (r != -1) {
            return mathutils.lineIntersectCircle(x0, y0, x1, y1, x, y, r);
        }
        else if (w != -1 && h != -1) {
	        return mathutils.lineIntersectRect(x0, y0, x1, y1, x, y, w, h);
        }
        else {
	        return mathutils.pointOnLine(x0, y0, x1, y1, x, y);
        }
    }
}