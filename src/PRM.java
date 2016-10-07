/**
 * Created by kdonahoe on 10/5/16.
 */

import com.sun.corba.se.impl.orbutil.graph.Graph;
import org.w3c.dom.css.Rect;

import java.awt.Rectangle;
import java.util.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import java.awt.Point;

public class PRM {
    ArrayList<Rectangle> walls;
    ArrayList<Point> samplePoints;
    ArrayList<Point2D.Double> joints;
    GeneralPath robot;
    World robotWorld;

    public PRM(World robotWorld) {
        this.robotWorld = robotWorld;
        this.robot = robotWorld.getRobot();
        this.walls = robotWorld.getWalls();
        this.joints = robotWorld.joints;
        this.samplePoints = robotWorld.getSamplePoints();

    }

    public void performPRM() {
        samplePoints = generateSamplePoints();
        robotWorld.updateWorldWithSampleNodes(robotWorld.getGraphics());
        localPlanner();

    }

    // sampling method
    public ArrayList<Point> generateSamplePoints() {
        Random rand = new Random();
        Point point;

        while(samplePoints.size() < 50) {
            point = new Point();
            point.x = rand.nextInt(600) + 1;
            point.y = rand.nextInt(600) + 1;

            // Makes sure that there are no collisions with walls, other points, or duplicate points.
            if(wallCollision(point) || closeToOtherPoint(point)) {
                System.out.println("Collided with wall or other point: " + point.x  + ", " + point.y);
                continue;
            }
            System.out.println("Didn't collide: " + point.x + ", " + point.y);
            samplePoints.add(point);
        }

        return samplePoints;
    }

    public boolean closeToOtherPoint(Point point) {
        for(Point existingPoint : samplePoints) {
            if ((Math.abs(point.x - existingPoint.x) < 3) && (Math.abs(point.y - existingPoint.y) < 3)) {
                return true;
            }
        }
        return false;
    }

    // Collision detection method that checks if the point collides with a wall.
    public boolean wallCollision(Point point) {
        ArrayList<Rectangle> walls = robotWorld.getWalls();
        for(Rectangle wall : walls) {
            // Add/subtract 10 to include a bit of a buffer to the edges of the walls to make up for the width of the points.
            if((point.x > wall.getX() - 10 && point.x <= (wall.getX() + wall.getWidth() + 10)) &&
                    (point.y > wall.getY() - 10 && point.y <= (wall.getY() + wall.getHeight() + 10))) {
                return true;
            }
        }
        return false;
    }

    // local planner:
    // find their k nearest neighbors
        // for each neighbor
            // see if a link connecting the node with that neighbor would be a collision
                // if not, create a link
    public void localPlanner() {
        // graph that maps vertex index to the node itself
            // a node contains the point, and all its neighbors
        HashMap<Integer, PRMNode> graph = new HashMap();

        int size = 10;
        // first get the k-neighbors of each of the initial joints
        for(Point2D.Double joint : joints) {
            ArrayList<Point> neighbors = findKNeighbors(joint);
            robotWorld.updateWorldWithKNeighbors(robotWorld.getGraphics(), joint, neighbors, size);
            size+=5;
        }

        /// then get the k-neighbors of each of the sample points
        for(Point point : samplePoints) {
            Point2D.Double converted = new Point2D.Double(point.x, point.y);
            ArrayList<Point> neighbors = findKNeighbors(converted);
            robotWorld.updateWorldWithKNeighbors(robotWorld.getGraphics(), converted, neighbors, size);
        }
    }

    // Given a vertex, finds tke k-closest neighboring vertices, and returns those k-neighbors
    // in an ArrayList.
    // The k-closest neighbors are measured by the euclidean distance between the two vertices.
    // Additionally, a vertex is only added as a neighbor to another vertex if the link connecting
    // the two vertices does not collide with a wall.
    public ArrayList<Point> findKNeighbors(Point2D.Double joint) {
        ArrayList<Point> neighbors = new ArrayList<>();
        int k = 0;

        TreeMap map = new TreeMap();
        for(Point vertex : samplePoints) {
            Double xDist = Math.abs(vertex.getX() - joint.getX());
            Double yDist = Math.abs(vertex.getY() - joint.getY());
            Double distance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
            map.put(distance, vertex);
        }

        Set neighborSet = map.entrySet();
        Iterator it = neighborSet.iterator();

        while(it.hasNext()) {
            if(k == 4) {
                break;
            }
            Map.Entry me = (Map.Entry) it.next();
            Point neighbor = (Point) me.getValue();
            if(linkCollidesWithWall(joint, neighbor)) {
                continue;
            }
            neighbors.add(neighbor);
            k++;
        }
        return neighbors;
    }

    // Checks to make sure that a link connecting two points don't collide with a wall.
    public boolean linkCollidesWithWall(Point2D.Double joint, Point point) {
        for(Rectangle wall : walls) {
            if(wall.intersectsLine(joint.x, joint.y, point.x, point.y)) {
                return true;
            }
        }
        return false;
    }

}
