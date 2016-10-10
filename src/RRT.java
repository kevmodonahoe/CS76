/**
 * Created by kdonahoe on 10/10/16.
 */

import com.sun.corba.se.impl.orbutil.graph.Graph;
import org.w3c.dom.css.Rect;
import java.awt.*;
import java.awt.Rectangle;
import java.util.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import javafx.scene.shape.Circle;

public class RRT {
    MobileRobotWorld robotWorld;
    // maps node to it's closest neighbor
    HashMap<Point2D.Double, Point2D.Double> graph;

    public RRT(MobileRobotWorld robotWorld) {
        this.robotWorld = robotWorld;
        this.graph = new HashMap<>();
    }

    public void performRRT() {
        ArrayList<Point2D.Double> samplePoints = generateSamplePoints();
        generateGraph(samplePoints);
    }

    public ArrayList<Point2D.Double> generateSamplePoints() {
        ArrayList<Point2D.Double> samplePoints = new ArrayList<>();
        Random rand = new Random();
        Point2D.Double point = new Point2D.Double(400, 400);
        samplePoints.add(point);

        while(samplePoints.size() < 2500) {
            point = new Point2D.Double();
            point.x = rand.nextInt(800) - 0;
            point.y = rand.nextInt(800) - 0;

            // Makes sure that there are no collisions with walls, other points, or duplicate points.
//            if(wallCollision(point) || closeToOtherPoint(point)) {
//                continue;
//            }
            samplePoints.add(point);
        }

        return samplePoints;
    }

    public void generateGraph(ArrayList<Point2D.Double> samplePoints) {
        for(Point2D.Double point : samplePoints) {
            Point2D.Double closestNeighbor = getClosestNeighbor(point);
            robotWorld.updateWorldWithLink(robotWorld.getGraphics(), point, closestNeighbor);
            System.out.println("closest neighbor of: " + point + " is: " + closestNeighbor);
        }
    }

    public Point2D.Double getClosestNeighbor(Point2D.Double point) {
        double smallestDistance = 800;
        Point2D.Double closestNeighbor = null;

        if(graph.size() == 0) {
            graph.put(point, point);
            return point;
        }

        Set mapSet = graph.entrySet();
        Iterator it = mapSet.iterator();

        while(it.hasNext()) {
            Map.Entry me = (Map.Entry) it.next();
            Point2D.Double checkNode = (Point2D.Double) me.getKey();

            if(getDistance(checkNode, point) < smallestDistance) {
                smallestDistance = getDistance(checkNode, point);
                closestNeighbor = checkNode;
            }
        }
        graph.put(point, closestNeighbor);
        return closestNeighbor;
    }

    public double getDistance(Point2D.Double firstNode, Point2D.Double secondNode) {
        Double xDist = Math.abs(firstNode.getX() - secondNode.getX());
        Double yDist = Math.abs(firstNode.getY() - secondNode.getY());
        Double distance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        return distance;
    }

}
