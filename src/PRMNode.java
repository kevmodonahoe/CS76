/**
 * Created by kdonahoe on 10/5/16.
 */

import java.util.ArrayList;
import java.awt.Point;


// NOT USING THIS RIGHT NOW

public class PRMNode {
    ArrayList<PRMNode> adjacencyNodes;
    Point p;
    int index;

    public PRMNode() {
        adjacencyNodes = new ArrayList<>();
        p = new Point();
    }

    public ArrayList<PRMNode> getAdjacencyNodes() {
        return adjacencyNodes;
    }

    public void setAdjacencyNodes(ArrayList<PRMNode> adjacencyNodes) {
        this.adjacencyNodes = adjacencyNodes;
    }

    public Point getP() {
        return p;
    }

    public void setP(Point p) {
        this.p = p;
    }


}
