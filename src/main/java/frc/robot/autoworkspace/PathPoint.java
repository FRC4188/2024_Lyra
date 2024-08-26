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
    
    boolean isSamePoint(PathPoint a) {
        return a.translation == this.translation;
    }
}
