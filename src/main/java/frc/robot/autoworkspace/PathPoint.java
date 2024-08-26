package frc.robot.autoworkspace;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPoint {
    Translation2d translation;
    double disFromEnd;
    PathPoint nextPoint;

    PathPoint(Translation2d point, double disFromEnd, PathPoint next) {
        translation = point;
        this.disFromEnd = disFromEnd;
        nextPoint = next;
    }
    
    boolean isInRange(double range, PathPoint a) {
        
        return a != null && (
            mathutils.distancePoints(translation.getX(), 
                                    translation.getY(), 
                                    a.translation.getX(), 
                                    a.translation.getY()) <= range
                                    );
    }
}
