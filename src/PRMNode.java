/**
 * Created by kdonahoe on 10/5/16.
 */

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.awt.Point;


// NOT USING THIS RIGHT NOW

public class PRMNode {
    ArrayList<PRMNode> adjacencyNodes;
    Point2D.Double p;
    int index;

    public PRMNode() {
        adjacencyNodes = new ArrayList<>();
        p = new Point2D.Double();
    }

    public ArrayList<PRMNode> getAdjacencyNodes() {
        return adjacencyNodes;
    }

    public void setAdjacencyNodes(ArrayList<PRMNode> adjacencyNodes) {
        this.adjacencyNodes = adjacencyNodes;
    }

    public Point2D.Double getP() {
        return p;
    }

    public void setP(Point2D.Double p) {
        this.p = p;
    }


}
