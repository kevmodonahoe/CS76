/**
 * Created by kdonahoe on 10/5/16.
 */

import com.sun.corba.se.impl.orbutil.graph.Graph;
import org.w3c.dom.css.Rect;
import java.awt.*;
import java.awt.Rectangle;
import java.util.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import javafx.scene.shape.Circle;


public class PRM {
    ArrayList<Rectangle> rectWalls;
    ArrayList<Circle> circularWalls;
    ArrayList<Point2D.Double> samplePoints;
    ArrayList<Point2D.Double> joints;
    ArrayList<Point2D.Double> goalJoints;
    ArrayList<Integer> angles;
    Point2D.Double basePoint;

    GeneralPath robot;
    World robotWorld;
    HashMap<State, ArrayList<State>> graph;

    public PRM(World robotWorld) {
        graph = new HashMap<>();
        this.basePoint = new Point2D.Double(400, 400);
        this.robotWorld = robotWorld;
        this.robot = robotWorld.getRobot();
        this.rectWalls = robotWorld.getRectWalls();
        this.joints = robotWorld.joints;
        this.goalJoints = robotWorld.goalJoints;
        this.samplePoints = new ArrayList<>();
    }

    public void performPRM() {
        ArrayList<State> randomSamples = generateSampleAngles();
        localPlanner(randomSamples);
        queryGraph();
    }

    public ArrayList<State> generateSampleAngles() {
        ArrayList<State> samples = new ArrayList<>();
        Random rand = new Random();

        while(samples.size() < 2500) {
            State sample = new State();
            int angle1 = rand.nextInt(360) - 0;
            int angle2 = rand.nextInt(360) - 0;
            int angle3 = rand.nextInt(360) - 0;

            sample.angles.add(angle1);
            sample.angles.add(angle2);
            sample.angles.add(angle3);

            ArrayList<Point2D.Double> points = robotWorld.generateJoints(basePoint, sample.angles);
            boolean collided = false;
            // checks to make sure the arm connecting this new set of angles wouldn't collide with a wall
            for(int i=0; i<points.size() - 1; i++) {
                if(i == 0) {
                    if(linkCollidesWithWall(basePoint, points.get(i))) {
                        collided = true;
                    }
                } else {
                    if(linkCollidesWithWall(points.get(i), points.get(i + 1))) {
                        collided = true;
                    }
                }
            }
            if(collided) {
                continue;
            }
            samples.add(sample);
        }
        return samples;
    }

    // Collision detection method that checks if the point collides with a wall.
    public boolean wallCollision(Point2D.Double point) {
        ArrayList<Rectangle> walls = robotWorld.getRectWalls();
        for(Rectangle wall : walls) {
            // Add/subtract 10 to include a bit of a buffer to the edges of the walls to make up for the width of the points.
            if((point.x > wall.getX() - 10 && point.x <= (wall.getX() + wall.getWidth() + 10)) &&
                    (point.y > wall.getY() - 10 && point.y <= (wall.getY() + wall.getHeight() + 10))) {
                return true;
            }
        }
        return false;
    }


    public void localPlanner(ArrayList<State> randomSamples) {
        randomSamples.add(robotWorld.startingConfig);
        randomSamples.add(robotWorld.goalConfig);

        for(State sample : randomSamples) {
            ArrayList<State> neighbors = findKNeighbors(sample, randomSamples);
            graph.put(sample, neighbors);
        }
    }

    public void queryGraph() {
        ArrayList<State> finalPath = breadthFirstSearch(robotWorld.startingConfig);
        if(finalPath != null) {
            System.out.println("Found the goal! Number of movements: " + finalPath.size());
            for(State config : finalPath) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                ArrayList<Point2D.Double> points = robotWorld.generateJoints(basePoint, config.angles);
                robotWorld.updateWorldWithSampleNodes(robotWorld.getGraphics(), points);
                robotWorld.updateWorldConnectSampleNodes(robotWorld.getGraphics(), points);
            }

        } else {
            System.out.println("Didnt find the goal");
        }
    }


    public ArrayList<State> breadthFirstSearch(State startConfig) {
        Queue<State> queue = new LinkedList();

        queue.add(startConfig);
        HashMap<State, State> visited = new HashMap();
        visited.put(startConfig, startConfig);
        while (!queue.isEmpty()) {

            State currConfig = queue.poll();
            if (goalTest(currConfig)) {
                ArrayList<State> finalPath = backchain(currConfig, visited);
                return finalPath;
            }

            ArrayList<State> neighbors = graph.get(currConfig);

            if(neighbors == null) {
                continue;
            }
            for(State neighbor : neighbors) {
                if (!visited.containsKey(neighbor)) {
                    queue.add(neighbor);
                    visited.put(neighbor, currConfig);
                }
            }
        }
        return null;
    }


    // backchain should only be used by bfs, not the recursive dfs
    private ArrayList<State> backchain(State node, HashMap<State, State> visited) {
        ArrayList<State> finalPath = new ArrayList<State>();
        finalPath.add(node);
        State parent = visited.get(node);

        while (visited.containsKey(parent)) {
            finalPath.add(parent);

            // the backchaining will have reached the beginning once the key and value are equal
            // we want to be sure to add this node, but only once
            if (parent == visited.get(parent)) {
                break;
            }
            parent = visited.get(parent);
        }
        Collections.reverse(finalPath);
        return finalPath;
    }

    // Goal test to see whether the current configuration is the goal configuration
    public boolean goalTest(State currConfig) {
        return (currConfig.angles == robotWorld.goalConfig.angles);
    }


    // Given a vertex, finds tke k-closest neighboring vertices, and returns those k-neighbors
    // in an ArrayList.
    // The k-closest neighbors are measured by the euclidean distance between the two vertices.
    // Additionally, a vertex is only added as a neighbor to another vertex if the link connecting
    // the two vertices does not collide with a wall.
    public ArrayList<State> findKNeighbors(State config, ArrayList<State> samples) {
        ArrayList<State> neighbors = new ArrayList<>();
        int k = 0;

        TreeMap map = new TreeMap();

        for(State sample : samples) {
            Double difference = 0.0;
            for(int i=0; i<sample.angles.size(); i++) {
                difference += Math.abs(sample.angles.get(i) - config.angles.get(i));
            }
            map.put(difference, sample);
        }

        Set neighborSet = map.entrySet();
        Iterator it = neighborSet.iterator();

        while(it.hasNext()) {
            if(k == 5) {
                break;
            }
            Map.Entry me = (Map.Entry) it.next();
            State neighbor = (State) me.getValue();

            // don't add the configuration as a neighbor if it is simply the current configuration itself
            if((config.equals(neighbor))) {
                continue;
            }
            neighbors.add(neighbor);
            k++;

        }
        return neighbors;


    }

    // Checks to make sure that a link connecting two points don't collide with a wall.
    public boolean linkCollidesWithWall(Point2D.Double joint, Point2D.Double point) {
        for(Rectangle wall : rectWalls) {
            if(wall.intersectsLine(joint.x, joint.y, point.x, point.y)) {
                return true;
            }
        }
        return false;
    }

}
