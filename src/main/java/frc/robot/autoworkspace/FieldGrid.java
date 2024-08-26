package frc.robot.autoworkspace;
import java.util.ArrayList;

class FieldNode {
    boolean obstacle = false, visited = false;
    double fGlobalGoal, fLocalGoal;
    short x, y;
    ArrayList<FieldNode> neighbours;
    FieldNode parent;
}

public class FieldGrid {
    FieldNode[] nodes = null;
    short gridWidth, gridHeight;
    double gridxScale, gridyScale;

    FieldNode startNode = null;
    FieldNode endNode = null;

    ArrayList<FieldNode> pivots;

    FieldObjects fobjs = null;

    FieldGrid(double width, double height, double sampleSize, FieldObjects fobjs) {
        gridxScale = Math.min(sampleSize, width);
        gridyScale = Math.min(sampleSize, height);

        gridWidth = (short) (width / gridxScale);
        gridHeight = (short) (height / gridyScale);

        gridxScale = width / gridWidth;
        gridyScale = height / gridHeight;

        this.fobjs = fobjs;

        createGrid(gridWidth, gridHeight);
    }

    void createGrid(short w, short h) {
        nodes = new FieldNode[w  * h];

        for (short x = 0; x < w; x++) {
            for (short y = 0; y < h; y++) {
                nodes[y * h + x].x =x;
                nodes[y * h + x].y =y;
                nodes[y * h + x].obstacle = (fobjs == null)?false:fobjs.pointIsTouchingAny(x * gridxScale + 0.5 * gridxScale, y * gridyScale + 0.5 * gridyScale);
                nodes[y * h + x].parent = null;
                nodes[y * h + x].visited = false;
                nodes[y * h + x].neighbours = new ArrayList<FieldNode>();
            }
        }

        for (short x = 0; x < w; x++) {
            for (short y = 0; y < h; y++) {
                if (y > 0) nodes[y * w + x].neighbours.add(nodes[(y - 1) * w + (x + 0)]);
                if (y < h - 1) nodes[y * w + x].neighbours.add(nodes[(y + 1) * w + (x + 0)]);
                if (x > 0) nodes[y * w + x].neighbours.add(nodes[(y + 0) * w + (x - 1)]);
                if (x < w - 1) nodes[y * w + x].neighbours.add(nodes[(y + 0) * w + (x + 1)]);

                if (y > 0 && x > 0) nodes[y * w + x].neighbours.add(nodes[(y - 1) * w + (x - 1)]);
                if (y > 0 && x < w - 1) nodes[y * w + x].neighbours.add(nodes[(y - 1) * w + (x + 1)]);
                if (y < h - 1 && x < w - 1) nodes[y * w + x].neighbours.add(nodes[(y + 1) * w + (x + 1)]);
                if (y < h - 1 && x > 0) nodes[y * w + x].neighbours.add(nodes[(y + 1) * w + (x -1)]);
            }
        }
    }


    void setStart(short x, short y) {
        x = mathutils.clamp((short)0, x, (short)(gridWidth - 1));
        y = mathutils.clamp((short)0, y, (short)(gridHeight - 1));

        startNode = nodes[y * gridWidth + x];

    }

    void setEnd(short x, short y) {
        x = mathutils.clamp((short)0, x, (short)(gridWidth - 1));
        y = mathutils.clamp((short)0, y, (short)(gridHeight - 1));

        endNode = nodes[y * gridWidth + x];
    }

    double heuristic(FieldNode a, FieldNode b) {
        return mathutils.distancePoints(a.x, a.y, b.x, b.y);
    }

    void resetEnds() {
        startNode = null;
        endNode = null;
    }

    void resetPathFind() {
        for (short x = 0; x < gridWidth; x++) {
	        for (short y = 0; y < gridHeight; y++) {
		        nodes[y * gridWidth + x].visited = false;
		        nodes[y * gridWidth + x].fGlobalGoal = Double.MAX_VALUE;
		        nodes[y * gridWidth + x].fLocalGoal = Double.MAX_VALUE;
		        nodes[y * gridWidth + x].parent = null;
	        }
        }
    }

    boolean pathFind() {
        resetPathFind();

        if (startNode == null || endNode == null) return false;

        FieldNode curNode = startNode;
        startNode.fLocalGoal = 0.0f;
        startNode.fGlobalGoal = heuristic(startNode, endNode);

        ArrayList<FieldNode> listNotTestedNodes = new ArrayList<FieldNode>();
        listNotTestedNodes.add(startNode);

        while (!listNotTestedNodes.isEmpty() && curNode != endNode) {
            listNotTestedNodes.sort((FieldNode lhs, FieldNode rhs) -> Double.compare(lhs.fGlobalGoal, rhs.fGlobalGoal));

            while (!listNotTestedNodes.isEmpty() && listNotTestedNodes.get(0).visited) {
                    listNotTestedNodes.remove(0);
            }

            if (listNotTestedNodes.isEmpty()) break;

            curNode = listNotTestedNodes.get(0);
            curNode.visited = true;

            for (FieldNode nodeNeighbour : curNode.neighbours) {
                if (!nodeNeighbour.visited && nodeNeighbour.obstacle) listNotTestedNodes.add(nodeNeighbour);

                double fPossiblyLowerGoal = curNode.fLocalGoal + heuristic(curNode, nodeNeighbour);

                if (fPossiblyLowerGoal < nodeNeighbour.fLocalGoal) {
                    nodeNeighbour.parent = curNode;
                    nodeNeighbour.fLocalGoal = fPossiblyLowerGoal;

                    nodeNeighbour.fGlobalGoal = nodeNeighbour.fLocalGoal + heuristic(nodeNeighbour, endNode);
                }

            }
        }

        return endNode.parent == null;
    }

    boolean createPivots() {
        pivots.clear();

        if (endNode == null || endNode.parent == null) return false;
        
        pivots.add(endNode);
        pivots.add(startNode);


        FieldNode curNode = null;
        FieldNode pivotNode = null;
        int curIndex = 0;

        while (curIndex < pivots.size() - 1) {
            curNode = pivots.get(curIndex).parent;
            
            if (lineTouchingAnyFF(pivots.get(curIndex), pivots.get(curIndex + 1))) {

                double maxD = -1;

                while (curNode != pivots.get(curIndex + 1)) {

                    double d = pointFromLineFF(pivots.get(curIndex), pivots.get(curIndex + 1), curNode);

                    if (maxD < d) {
                        maxD = d;
                        pivotNode = curNode;
                    }

                    curNode = curNode.parent;
                }

                if (pivotNode != null) {
                    pivots.add(curIndex + 1, pivotNode);
                } else {
                    curIndex++;
                }

            } else {
                curIndex++;
            }
        }
        return true;
    }

    boolean lineTouchingAnyFF(FieldNode a, FieldNode b) {
        return fobjs.lineIsTouchingAny(a.x * gridxScale + 0.5 * gridxScale, a.y * gridyScale + 0.5 * gridyScale, b.x * gridxScale + 0.5 *gridxScale, b.y * gridyScale + 0.5 * gridyScale );
    }

    double pointFromLineFF(FieldNode l1, FieldNode l2, FieldNode p) {
        return mathutils.pointFromLine(l1.x, l1.y, l2.x, l2.y, p.x, p.y);
    }
}
