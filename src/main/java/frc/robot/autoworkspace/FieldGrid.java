package frc.robot.autoworkspace;
import java.util.ArrayList;

class FieldNode {
    //A start algorithm purposes
    boolean obstacle = false, visited = false;
    double fGlobalGoal, fLocalGoal;
    short x, y;
    ArrayList<FieldNode> neighbours;
    FieldNode parent;
}

public class FieldGrid {
    //Grid of nodes
    FieldNode[] nodes = null;
    short gridWidth, gridHeight;
    //grid to scale
    double gridxScale, gridyScale;

    //start and end nodes
    FieldNode startNode = null;
    FieldNode endNode = null;

    //list of pivots (backwards)
    ArrayList<FieldNode> pivots;

    //field objects references
    FieldObjects fobjs = null;

    //constructor field grid, based on units of measure
    FieldGrid(double width, double height, double sampleSize, FieldObjects fobjs) {
        //ensures sample size does not exceded width and height
        gridxScale = Math.min(sampleSize, width);
        gridyScale = Math.min(sampleSize, height);

        //creates grid width and height rounded down from sample size
        gridWidth = (short) (width / gridxScale);
        gridHeight = (short) (height / gridyScale);

        //sets scale again so that it matches the rounded grid width and height
        gridxScale = width / gridWidth;
        gridyScale = height / gridHeight;

        this.fobjs = fobjs;

        pivots = new ArrayList<FieldNode>();

        //creates grid with field objects in mind
        createGrid(gridWidth, gridHeight);
    }

    void createGrid(short w, short h) {
        //initislizes grid array
        gridWidth = w;
        gridHeight = h;
        nodes = new FieldNode[w  * h];

        //initlizes individial nodes
        for (short x = 0; x < w; x++) {
            for (short y = 0; y < h; y++) {
                nodes[y * w + x] = new FieldNode();
                nodes[y * w + x].x =x;
                nodes[y * w + x].y =y;
                nodes[y * w + x].obstacle = (fobjs == null)?false:fobjs.pointIsTouchingAny(x * gridxScale + 0.5 * gridxScale, y * gridyScale + 0.5 * gridyScale);
                nodes[y * w + x].parent = null;
                nodes[y * w + x].visited = false;
                nodes[y * w + x].neighbours = new ArrayList<FieldNode>();
            }
        }

        //set neighbor nodes, both cardinal and diagonal
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

    //sets start node (according to grid units)
    void setStart(short x, short y) {
        x = mathutils.clamp((short)0, x, (short)(gridWidth - 1));
        y = mathutils.clamp((short)0, y, (short)(gridHeight - 1));

        startNode = nodes[y * gridWidth + x];

    }

    //sets end node (based on grid units)
    void setEnd(short x, short y) {
        x = mathutils.clamp((short)0, x, (short)(gridWidth - 1));
        y = mathutils.clamp((short)0, y, (short)(gridHeight - 1));

        endNode = nodes[y * gridWidth + x];
    }

    //heuristic between nodes
    double heuristic(FieldNode a, FieldNode b) {
        return mathutils.distancePoints(a.x, a.y, b.x, b.y);
    }

    //sets start and end to null
    void resetEnds() {
        startNode = null;
        endNode = null;
    }

    //resets path finding
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

    //path finds through grid, return true if sucessful
    boolean pathFind() {
        //resets
        resetPathFind();

        //if ends are not set, return false
        if (startNode == null || endNode == null) return false;

        //starts on the stsart
        FieldNode curNode = startNode;
        startNode.fLocalGoal = 0.0f;
        startNode.fGlobalGoal = heuristic(startNode, endNode);

        //list of not tested nodes
        ArrayList<FieldNode> listNotTestedNodes = new ArrayList<FieldNode>();
        listNotTestedNodes.add(startNode);

        //path finds until list of not tested nodes are empty or end node has been reached
        while (!listNotTestedNodes.isEmpty() && curNode != endNode) {
            //sorts the list based on lowest to highest global goal
            listNotTestedNodes.sort((FieldNode lhs, FieldNode rhs) -> Double.compare(lhs.fGlobalGoal, rhs.fGlobalGoal));

            //removes first node if it has been tested aleady
            while (!listNotTestedNodes.isEmpty() && listNotTestedNodes.get(0).visited) {
                    listNotTestedNodes.remove(0);
            }

            //if empty then break loop
            if (listNotTestedNodes.isEmpty()) break;

            //curNode is set to the start and sets tested to true
            curNode = listNotTestedNodes.get(0);
            curNode.visited = true;

            //iterates through the neighbours of curNode
            for (FieldNode nodeNeighbour : curNode.neighbours) {
                //if the neighbor has not been visited nor is an obstacle, add it to the list of not tested nodes
                if (!nodeNeighbour.visited && !nodeNeighbour.obstacle) listNotTestedNodes.add(nodeNeighbour);

                //a start math shi i forgor
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

    /* 
    creates pivots of the path based on the path
    creates lines between neighbouring pivots
    if the line crosses and object, iterates through the field nodes of the path
    finding the farthest away from said line, and setting that as a pivot between those pivots, 
    then repeat

    creates it backward because the it can only iterate through the path backwards (end to start)

    returns true if pivots successful
    */
    boolean createPivots() {
        //clear pivots
        pivots.clear();

        //if start and end point do not exist nor does that path, return false
        if (endNode == null || endNode.parent == null) return false;
        
        //sets end and start as pivots
        pivots.add(endNode);
        pivots.add(startNode);

        //curNode is iteratating node, pivot node is the node away from the line and will be added to the list of pivots
        FieldNode curNode = null;
        FieldNode pivotNode = null;
        //indexes through nodes
        int curIndex = 0;

        //until the index reaches the last pivot
        while (curIndex < pivots.size() - 1) {
            //curNode is the node after the pivot
            curNode = pivots.get(curIndex).parent;
            
            //if line crosses obstacle, else curIndex++
            if (lineTouchingAnyFF(pivots.get(curIndex), pivots.get(curIndex + 1))) {

                //max distance initlized
                double maxD = -1;

                //iterates until curNode hits the next pivot
                while (curNode != pivots.get(curIndex + 1)) {

                    //measures distance between field node and pivot line
                    double d = pointFromLineFF(pivots.get(curIndex), pivots.get(curIndex + 1), curNode);

                    //if distance is greater than max, set pivot node to curNode, and set max to distance
                    if (maxD < d) {
                        maxD = d;
                        pivotNode = curNode;
                    }

                    //next node
                    curNode = curNode.parent;
                }

                //if pivot node is null, curIndex++, else insert to list
                if (pivotNode != null) {
                    pivots.add(curIndex + 1, pivotNode);
                } else {
                    curIndex++;
                }

            } else {
                curIndex++;
            }
        }
        //if it reaches the end return true (created pivots)
        return true;
    }

    //checks if line crosses any obstacles
    boolean lineTouchingAnyFF(FieldNode a, FieldNode b) {
        return fobjs.lineIsTouchingAny(a.x * gridxScale + 0.5 * gridxScale, a.y * gridyScale + 0.5 * gridyScale, b.x * gridxScale + 0.5 *gridxScale, b.y * gridyScale + 0.5 * gridyScale );
    }

    //measures point from line
    double pointFromLineFF(FieldNode l1, FieldNode l2, FieldNode p) {
        return mathutils.pointFromLine(l1.x, l1.y, l2.x, l2.y, p.x, p.y);
    }
}
