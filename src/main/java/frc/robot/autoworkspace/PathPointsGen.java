package frc.robot.autoworkspace;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class PathPointsGen {
    public List<Translation2d> pivots;
    private Grid grid = null;
    public FieldObjectHandler fobjs = null;

    public PathPointsGen(double length, double width, double sampleSize, FieldObjectHandler fobjs) {
        grid = new Grid(length, width, sampleSize, fobjs);
        this.fobjs = (fobjs == null)?new FieldObjectHandler():fobjs;
        pivots = new ArrayList<Translation2d>();
    }

    //generation methods
    private boolean generatePivots(Translation2d start, Translation2d end) {
        List<Node> backwardPivots = new ArrayList<Node>();
        pivots.clear();

        if (!grid.pathFind(t2dToNode(start), t2dToNode(end))) return false;

        backwardPivots.add(grid.endNode);
        backwardPivots.add(grid.startNode);

        Node curNode = null;
        Node pivotNode = null;
        int curIndex = 0;

        while (curIndex < backwardPivots.size() - 1) {

            curNode = backwardPivots.get(curIndex).parent;

            if (lineIsTouchingAny(backwardPivots.get(curIndex), backwardPivots.get(curIndex + 1))) {
                double maxD = -1;

                while (curNode != backwardPivots.get(curIndex + 1)) {
                    double d = pointFromLine(backwardPivots.get(curIndex), backwardPivots.get(curIndex + 1), curNode);

                    if (maxD < d) {
                        maxD = d;
                        pivotNode = curNode;
                    }

                    curNode = curNode.parent;
                }

                if (pivotNode != null) {
                    backwardPivots.add(curIndex + 1, pivotNode);
                } else {
                    curIndex++;
                }

            } else {
                curIndex++;
            }
        }

        Collections.reverse(backwardPivots);

        pivots = nodesToT2ds(backwardPivots);
        
        if (pivots.size() < 2) return false;

        pivots.remove(0);
        pivots.remove(pivots.size() - 1);

        return pivots.size() > 1;
    }

    public Trajectory generateTrajectory(Translation2d start, Translation2d end) {
        if (!generatePivots(start, end)) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 2, Constants.drivetrain.MAX_ACCEL / 2));
    }

    public Trajectory generateTrajectory(Pose2d start, Translation2d end) {
        if (!generatePivots(start.getTranslation(), end)) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 2, Constants.drivetrain.MAX_ACCEL / 2));
    }

    public Trajectory generateTrajectory(Translation2d start, Pose2d end) {
        if (!generatePivots(start, end.getTranslation())) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            end,
            new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 2, Constants.drivetrain.MAX_ACCEL / 2));
    }

    public Trajectory generateTrajectory(Pose2d start, Pose2d end) {
        if (!generatePivots(start.getTranslation(), end.getTranslation())) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            end, 
            new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 2, Constants.drivetrain.MAX_ACCEL / 2));
    }

    //utility methods
    private List<Translation2d> nodesToT2ds(List<Node> nodes) {
        List<Translation2d> poses = new ArrayList<Translation2d>();

        for (Node n : nodes) {
            poses.add(nodeToT2d(n));
        }   

        return poses;
    }

    private Translation2d nodeToT2d(Node node) {
        return new Translation2d(grid.getNodeLength() * (node.getX() - 0.5), grid.getNodeWidth() * (node.getY() - 0.5));
    }

    private Node t2dToNode(Translation2d pose) {
        return grid.getNode((int) (pose.getX()/grid.getNodeLength() + 1), (int) (pose.getY()/grid.getNodeWidth() + 1));
    }

    private boolean lineIsTouchingAny(Node n1, Node n2) {
        return fobjs.lineIsTouchingAny(
            nodeToT2d(n1).getX(), nodeToT2d(n1).getY(), 
            nodeToT2d(n2).getX(), nodeToT2d(n2).getY()
        );
    }

    private double pointFromLine(Node n1, Node n2, Node point) {
        return mathutils.pointFromLine(
            nodeToT2d(n1).getX(), nodeToT2d(n1).getY(), 
            nodeToT2d(n2).getX(), nodeToT2d(n2).getY(),
            nodeToT2d(point).getX(), nodeToT2d(point).getY()  
        );
    }
}

//A star grid calculation
class Node {
    private short x, y;
    public List<Node> neighbours;
    public boolean obstacle = false, visited = false;
    public double fGlobalGoal, fLocalGoal;
    public Node parent;

    //constructors
    public Node(short x, short y) {
        this.x = x;
        this.y = y;
        neighbours = new ArrayList<Node>();
        resetGoals();
    }

    public Node(short x, short y, boolean obstacle) {
        this(x, y);
        this.obstacle = obstacle;
    }
    //actions
    public void resetGoals() {
        parent = null;
        visited = false;
        fGlobalGoal = Double.MAX_VALUE;
        fLocalGoal = Double.MAX_VALUE;
    }

    public void addNeighbour(Node other) {
        if (!neighbours.contains(other) && other != this) {
            neighbours.add(other);
        }
    }

    //get methods
    public short getX() {
        return x;
    }
    
    public short getY() {
        return y;
    }
}   

class Grid {
    private Node[] nodes = null;

    private short gridLength, gridWidth;
    private double gridxScale, gridyScale;

    public Node startNode = null;
    public Node endNode = null;

    public Grid(double length, double width, double sampleSize, FieldObjectHandler fobjs) {
        gridxScale = Math.min(sampleSize, length);
        gridyScale = Math.min(sampleSize, width);

        gridLength = (short) (length / gridxScale);
        gridWidth = (short) (width / gridyScale);

        gridxScale = length / gridLength;
        gridyScale = width / gridWidth;

        createGrid(gridLength, gridWidth, fobjs);
    }

    //generation methods
    private void createGrid(short l, short w, FieldObjectHandler fobjs) {
        gridWidth = w;
        gridLength = l;
        nodes = new Node[w  * l];

        for (short x = 0; x < l; x++) {
            for (short y = 0; y < w; y++) {
                nodes[y * l + x] = new Node(x, y, 
                    (fobjs == null)?false:fobjs.pointIsTouchingAny((x + 0.5) * gridxScale, (y + 0.5) * gridyScale)
                );
            }
        }

        for (short x = 0; x < l; x++) {
            for (short y = 0; y < w; y++) {
                for (short nx = -1; nx < 2; nx++) {
                    for (short ny = -1; ny < 2; ny++) {
                        nodes[y * l + x].addNeighbour(getNode(x + nx, y + ny));
                    }
                }
            }
        }
    }

    public void resetEnds() {
        startNode = null;
        endNode = null;
    }

    private void resetPathFind() {
        for (Node n: nodes) {
            n.resetGoals();
        }
    }

    public boolean pathFind(Node start, Node end) {
        resetPathFind();
        startNode = start;
        endNode = end;

        if (startNode == null || endNode == null) return false;

        Node curNode = startNode;
        startNode.fLocalGoal = 0.0f;
        startNode.fGlobalGoal = heuristic(startNode, endNode);

        List<Node> listNotTestedNodes = new ArrayList<Node>();
        listNotTestedNodes.add(startNode);

        while (!listNotTestedNodes.isEmpty() && curNode != endNode) {
            listNotTestedNodes.sort((Node lhs, Node rhs) -> Double.compare(lhs.fGlobalGoal, rhs.fGlobalGoal));

            while (!listNotTestedNodes.isEmpty() && listNotTestedNodes.get(0).visited) {
                    listNotTestedNodes.remove(0);
            }

            if (listNotTestedNodes.isEmpty()) break;

            curNode = listNotTestedNodes.get(0);
            curNode.visited = true;

            for (Node nodeNeighbour : curNode.neighbours) {
                if (!nodeNeighbour.visited && !nodeNeighbour.obstacle) listNotTestedNodes.add(nodeNeighbour);

                double fPossiblyLowerGoal = curNode.fLocalGoal + heuristic(curNode, nodeNeighbour);

                if (fPossiblyLowerGoal < nodeNeighbour.fLocalGoal) {
                    nodeNeighbour.parent = curNode;
                    nodeNeighbour.fLocalGoal = fPossiblyLowerGoal;
                    nodeNeighbour.fGlobalGoal = nodeNeighbour.fLocalGoal + heuristic(nodeNeighbour, endNode);
                }

            }
        }

        //returns if path is successful
        return endNode.parent != null;
    }
    
    //get methods
    public int getGridWidth() {
        return gridWidth;
    }

    public int getGridLength() {
        return gridLength;
    }

    public double getNodeLength() {
        return gridxScale;
    }

    public double getNodeWidth() {
        return gridyScale;
    }

    public Node[] getNodes() {
        return nodes;
    }

    public Node getNode(int x, int y) {
        x = mathutils.clamp(0, x, gridLength - 1);
        y = mathutils.clamp(0, y, gridWidth - 1);
        return nodes[x + y * gridLength];
    }
    //util methods
    private double heuristic(Node a, Node b) {
        return mathutils.distancePoints(a.getX(), a.getY(), b.getX(), b.getY());
    }

}


