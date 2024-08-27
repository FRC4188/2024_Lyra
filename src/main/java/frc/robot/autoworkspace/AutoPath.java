package frc.robot.autoworkspace;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPath {
    //List of path points in order
    ArrayList<PathPoint> pathPoints;

    //field objects and field grid references
    FieldObjects fobjs = null;
    FieldGrid fg = null;

    //constructor
    AutoPath(FieldGrid fg) {
        this.fg = fg;
        this.fobjs = fg.fobjs;
        pathPoints = new ArrayList<PathPoint>();
    }

    //distance between PathPoints
    double distancePoints(PathPoint a, PathPoint b) {
        return mathutils.distancePoints(a.translation.getX(), a.translation.getY(), a.translation.getX(), a.translation.getY());
    }

    //distance between FieldNodes (in actual scale)
    double distanceNode(FieldNode a, FieldNode b) {
        return mathutils.distancePoints(
            a.x * fg.gridxScale + 0.5 * fg.gridxScale,
            a.y * fg.gridyScale + 0.5 * fg.gridyScale,
            b.x * fg.gridxScale + 0.5 * fg.gridxScale,
            b.y * fg.gridyScale + 0.5 * fg.gridyScale);
    }

    //interpolates based on a list of points, index for p1, and t value (between  0 and 1)
    Translation2d interpolate(ArrayList<FieldNode> points, int index, double t) {
        double x = mathutils.cubicInterpolation(
            points.get(Math.max(0, index - 1)).x * fg.gridxScale + 0.5 * fg.gridxScale, 
            points.get(Math.min(points.size() - 1, index)).x * fg.gridxScale + 0.5 * fg.gridxScale,
            points.get(Math.min(points.size() - 1, index + 1)).x * fg.gridxScale + 0.5 * fg.gridxScale,
            points.get(Math.min(points.size() - 1, index + 2)).x * fg.gridxScale + 0.5 * fg.gridxScale, t);
        
        double y = mathutils.cubicInterpolation(
            points.get(Math.max(0, index - 1)).y * fg.gridyScale + 0.5 * fg.gridyScale, 
            points.get(Math.min(points.size() - 1, index)).y * fg.gridyScale + 0.5 * fg.gridyScale,
            points.get(Math.min(points.size() - 1, index + 1)).y * fg.gridyScale + 0.5 * fg.gridyScale,
            points.get(Math.min(points.size() - 1, index + 2)).y * fg.gridyScale + 0.5 * fg.gridyScale, t);

        return new Translation2d(x, y);
    }

    //converts Field Node to a Translation2d Object
    Translation2d nodeToT2d(FieldNode point) {
        return new Translation2d(point.x * fg.gridxScale + 0.5 * fg.gridxScale, point.y * fg.gridyScale + 0.5 * fg.gridyScale);
    }


    //creates list of path points by cubic interpolation through the list of pivot nodes
    boolean createAutoPath() {
        //clears current list
        pathPoints.clear();
        //checks if pivots node list exists
        if (fg.pivots.size() == 0) return false;

        //iterates through the list of pivot nodes, creating them from the end point to the start point
        for (int i = 0; i < fg.pivots.size() - 1; i++) {
            //crates a divisor that distributes the points evenly based on distance and not proportion between pivots
            double dis = distanceNode(fg.pivots.get(i), fg.pivots.get(i + 1));
            int divisor = Math.max(1, (int) (0.1 * dis));

            //iterates through t values 0 through 1
            for (double t = 0; t < 1; t += ((double)1/divisor)) {
                //creates current Translation 2d, next PathPoint, and distance from end
                Translation2d cur = interpolate(fg.pivots, i , mathutils.clamp(0, t, 1));
                PathPoint next = null;
                double distance = 0;

                //if pathPoints are empty, leave next null and distance 0 to notate the end of the PathPoint list
                if (pathPoints.size() != 0) {
                    next = pathPoints.get(pathPoints.size() - 1);
                    distance = cur.getDistance(next.translation) + next.disFromEnd;
                }
                
                //creates path point
                PathPoint add = new PathPoint(cur, distance, next);
                
                //last check if pathpoint is too close to next path point it will not be created
                //(probably comes with flaws)
                if (!add.isInRange(5, next)) {
                    pathPoints.add(add);
                }
            }
        }
        
        //crates start point to ensure there is a start created
        Translation2d cur = nodeToT2d(fg.pivots.get(fg.pivots.size() - 1));
        PathPoint next = null;
        double distance = 0;
        if (pathPoints.size() != 0) {
            next = pathPoints.get(pathPoints.size() - 1);
            distance = cur.getDistance(next.translation) + next.disFromEnd;
        }
                
        PathPoint add = new PathPoint(cur, distance, next);
        
        //less tolerance for range because its the start point
        if (!add.isInRange(0.01, next)) {
            pathPoints.add(add);
        }

        //reverse points so its in actual order
        Collections.reverse(pathPoints);
        
        return true;
    }


}
