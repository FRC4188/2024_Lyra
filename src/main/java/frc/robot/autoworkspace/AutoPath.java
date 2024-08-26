package frc.robot.autoworkspace;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPath {
    ArrayList<PathPoint> pathPoints;

    FieldObjects fobjs = null;
    FieldGrid fg = null;

    AutoPath(FieldGrid fg) {
        this.fg = fg;
        this.fobjs = fg.fobjs;
        pathPoints = new ArrayList<PathPoint>();
    }

    double distancePoints(PathPoint a, PathPoint b) {
        return mathutils.distancePoints(a.translation.getX(), a.translation.getY(), a.translation.getX(), a.translation.getY());
    }

    double distanceNode(FieldNode a, FieldNode b) {
        return mathutils.distancePoints(
            a.x * fg.gridxScale + 0.5 * fg.gridxScale,
            a.y * fg.gridyScale + 0.5 * fg.gridyScale,
            b.x * fg.gridxScale + 0.5 * fg.gridxScale,
            b.y * fg.gridyScale + 0.5 * fg.gridyScale);
    }

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

    Translation2d nodeToT2d(FieldNode point) {
        return new Translation2d(point.x * fg.gridxScale + 0.5 * fg.gridxScale, point.y * fg.gridyScale + 0.5 * fg.gridyScale);
    }

    boolean createAutoPath() {
        pathPoints.clear();
        if (fg.pivots.size() == 0) return false;

        for (int i = 0; i < fg.pivots.size() - 1; i++) {
            double dis = distanceNode(fg.pivots.get(i), fg.pivots.get(i + 1));
            int divisor = Math.max(1, (int) (0.1 * dis));

            for (double t = 0; t < 1; t += ((double)1/divisor)) {
                Translation2d cur = interpolate(fg.pivots, i , mathutils.clamp(0, t, 1));
                PathPoint next = null;
                double distance = 0;
                if (pathPoints.size() != 0) {
                    next = pathPoints.get(pathPoints.size() - 1);
                    distance = cur.getDistance(next.translation) + next.disFromEnd;
                }
                
                PathPoint add = new PathPoint(cur, distance, next);
                
                
                if (!add.isInRange(5, next)) {
                    pathPoints.add(add);
                }
            }
        }
        
        
        Translation2d cur = nodeToT2d(fg.pivots.get(fg.pivots.size() - 1));
        PathPoint next = null;
        double distance = 0;
        if (pathPoints.size() != 0) {
            next = pathPoints.get(pathPoints.size() - 1);
            distance = cur.getDistance(next.translation) + next.disFromEnd;
        }
                
        PathPoint add = new PathPoint(cur, distance, next);
                
        if (!add.isInRange(5, next)) {
            pathPoints.add(add);
        }

        Collections.reverse(pathPoints);
        
        return true;
    }


}
