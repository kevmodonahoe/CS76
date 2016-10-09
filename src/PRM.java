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
    ArrayList<Point2D.Double> samplePoints;
    ArrayList<Point2D.Double> joints;
    ArrayList<Point2D.Double> goalJoints;
    GeneralPath robot;
    World robotWorld;
    HashMap<Point2D.Double, ArrayList<Point2D.Double>> graph;


    public PRM(World robotWorld) {
        graph = new HashMap<>();
        this.robotWorld = robotWorld;
        this.robot = robotWorld.getRobot();
        this.walls = robotWorld.getWalls();
        this.joints = robotWorld.joints;
        this.goalJoints = robotWorld.goalJoints;
        this.samplePoints = robotWorld.getSamplePoints();

    }

    public void performPRM() {
        samplePoints = generateSamplePoints();
        robotWorld.updateWorldWithSampleNodes(robotWorld.getGraphics());
        localPlanner();
        robot.closePath();
        robotWorld.drawRobot(robotWorld.getGraphics());
        queryGraph();

    }

    // sampling method
    public ArrayList<Point2D.Double> generateSamplePoints() {
        Random rand = new Random();
        Point2D.Double point;

        while(samplePoints.size() < 50) {
            point = new Point2D.Double();
            point.x = rand.nextInt(800) - 0;
            point.y = rand.nextInt(800) - 0;

            // Makes sure that there are no collisions with walls, other points, or duplicate points.
            if(wallCollision(point) || closeToOtherPoint(point)) {
//                System.out.println("Collided with wall or other point: " + point.x  + ", " + point.y);
                continue;
            }
//            System.out.println("Didn't collide: " + point.x + ", " + point.y);
            samplePoints.add(point);
        }

        return samplePoints;
    }

    public boolean closeToOtherPoint(Point2D.Double point) {
        for(Point2D.Double existingPoint : samplePoints) {
            if ((Math.abs(point.x - existingPoint.x) < 3) && (Math.abs(point.y - existingPoint.y) < 3)) {
                return true;
            }
        }
        return false;
    }

    // Collision detection method that checks if the point collides with a wall.
    public boolean wallCollision(Point2D.Double point) {
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

        samplePoints.addAll(joints);
        samplePoints.addAll(goalJoints);

        // graph that maps vertex index to the node itself
            // a node contains the point, and all its neighbors
        // first get the k-neighbors of each of the initial joints and insert those start joints into the graph
        for(Point2D.Double joint : joints) {
            ArrayList<Point2D.Double> neighbors = findKNeighbors(joint);
            graph.put(joint, neighbors);
            robotWorld.updateWorldWithKNeighbors(robotWorld.getGraphics(), joint, neighbors);
        }

        // then get the k-neighbors of each of the goal joints, and insert those goal joints into the graph
        for(Point2D.Double joint : goalJoints) {
            ArrayList<Point2D.Double> neighbors = findKNeighbors(joint);
            graph.put(joint, neighbors);
            robotWorld.updateWorldWithKNeighbors(robotWorld.getGraphics(), joint, neighbors);
        }

        // then get the k-neighbors of each of the sample points, and insert those sample points into the graph
        for(Point2D.Double point : samplePoints) {
            ArrayList<Point2D.Double> neighbors = findKNeighbors(point);
            graph.put(point, neighbors);
            robotWorld.updateWorldWithKNeighbors(robotWorld.getGraphics(), point, neighbors);
        }

        // right now, I am calculating all possible moves throughout the whole movement
            // an alternate would be to calculate K-nearest for one joint, move that joint to one of
            // them, and then re-calculate k-nearest and move it to one of those

    }

    public void queryGraph() {
        Point2D.Double startNode = new Point2D.Double();
        startNode = robotWorld.joints.get(3);
        System.out.println("goal: " + robotWorld.goalJoints);
        ArrayList<Point2D.Double> finalPath = breadthFirstSearch(startNode);
        if(finalPath != null) {
            System.out.println("Fond the Goal!");
            robotWorld.updateWorldFinalPath(finalPath, robotWorld.getGraphics());
        } else {
            System.out.println("Didn't find a goal path");
        }
    }
//
//    public List<Point2D.Double> aStar() {
//        List<State> finalPath = new ArrayList<State>();
//        Comparator<State> comparator = new CostComparator();
//        PriorityQueue<State> pqueue = new PriorityQueue<State>(comparator);
//
//        // processed --> keeps track of hashCode of Node -> node
//        // processed is actually processed it (checked if goal or gotten it's children)
//        // only processes nodes that have been popped off of the pqueue
//        HashMap<Integer, State> processed = new HashMap();
//        HashMap<Integer, State> seen = new HashMap();
//
//        //parentMap --> maps a node to its parent
//        HashMap<State, State> parentMap = new HashMap();
//        double[] config = robotWorld.startConfigWithCoords;
//
//        State startConfig = new State(config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7]);
//
//        parentMap.put(startConfig, startConfig);
//        startConfig.distance = 0;
//        startConfig.heuristic = heuristic(startConfig);
//        pqueue.add(startConfig);
//
//        seen.put(startConfig.hashCode(), startConfig);
//
//
//        while (!pqueue.isEmpty()) {
//            // Be sure to keep track of the nodes you have explored, because when adding nodes
//            // to the priority queue,
//            State currNode = pqueue.poll();
//
//            System.out.println();
//            System.out.println("Exploring in A*: " + currNode.toString());
//
//            // if the node has already been processed, do not process it again
//            if(processed.containsKey(currNode.hashCode())) {
//                System.out.println("Already processed " + currNode.toString());
//                continue;
//            } else {
//                if(goalTest(currNode)) {
//                    addRobot(currNode);
//                    finalPath = backchain(currNode, parentMap);
//                    return finalPath;
//                }
//                addRobot(currNode);
//                try {
//
//                    //sleep 5 seconds
//                    Thread.sleep(100);
//
//
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//
//
//                ArrayList<State> neighbors = getNeighbors(currNode);
//
//                for(State neighbor : neighbors) {
//                    neighbor.depth = currNode.depth + 1;
//
//                    if(!processed.containsKey(neighbor.hashCode()) && !pqueue.contains(neighbor)) {
//                        neighbor.heuristic = heuristic(neighbor);
//                        pqueue.add(neighbor);
//                        parentMap.put(neighbor, currNode);
//                    } else {
//                        if(!processed.containsKey(neighbor.hashCode()) && pqueue.contains(neighbor) ) {
//                            State existingNode = seen.get(neighbor.hashCode());
//                            // the new node already is in the queue, but has a shorter path than the existing node,
//                            // add it to the queue
//                            if(heuristic(existingNode) > heuristic(neighbor)) {
//                                neighbor.heuristic = this.heuristic(neighbor);
//                                pqueue.add(neighbor);
//                                parentMap.put(neighbor, currNode);
//                            } else {
//                                System.out.println("Node was already in explored/pqueue, but not shorter depth: " + neighbor);
//                            }
//                        }
//                    }
//                    seen.put(neighbor.hashCode(), neighbor);
//                }
//            }
//            processed.put(currNode.hashCode(), currNode);
//            removeRobot(currNode);
//        }
//
//        return finalPath;
//    }
//
//
    public ArrayList<Point2D.Double> breadthFirstSearch(Point2D.Double startNode) {
        Queue<Point2D.Double> queue = new LinkedList();

        int count = 0;

        queue.add(startNode);
        HashMap<Point2D.Double, Point2D.Double> visited = new HashMap();
        visited.put(startNode, startNode);
        while (!queue.isEmpty()) {

            Point2D.Double currNode = queue.poll();
            count++;
            if (goalTest(currNode)) {
                ArrayList<Point2D.Double> finalPath = backchain(currNode, visited);
                System.out.println(finalPath);
                return finalPath;
            }

            ArrayList<Point2D.Double> neighbors = graph.get(currNode);
            System.out.println("getting neighbors of " + currNode);
//            System.out.println(neighbors);
            if(neighbors == null) {
                continue;
            }
            for(Point2D.Double neighbor : neighbors) {
                if (!visited.containsKey(neighbor)) {
                    queue.add(neighbor);
                    visited.put(neighbor, currNode);
                }
            }
        }
        System.out.println(count);
        return null;
    }


    // backchain should only be used by bfs, not the recursive dfs
    private ArrayList<Point2D.Double> backchain(Point2D.Double node, HashMap<Point2D.Double, Point2D.Double> visited) {
        ArrayList<Point2D.Double> finalPath = new ArrayList<Point2D.Double>();
        finalPath.add(node);
        Point2D.Double parent = visited.get(node);

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

    public boolean goalTest(Point2D.Double currNode) {
        return (currNode.x == robotWorld.goalJoints.get(3).x && currNode.y == robotWorld.goalJoints.get(3).y);
    }


    // Given a vertex, finds tke k-closest neighboring vertices, and returns those k-neighbors
    // in an ArrayList.
    // The k-closest neighbors are measured by the euclidean distance between the two vertices.
    // Additionally, a vertex is only added as a neighbor to another vertex if the link connecting
    // the two vertices does not collide with a wall.
    public ArrayList<Point2D.Double> findKNeighbors(Point2D.Double joint) {
        ArrayList<Point2D.Double> neighbors = new ArrayList<>();
        int k = 0;

        TreeMap map = new TreeMap();

        for(Point2D.Double vertex : samplePoints) {
            Double xDist = Math.abs(vertex.getX() - joint.getX());
            Double yDist = Math.abs(vertex.getY() - joint.getY());
            Double distance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
            map.put(distance, vertex);
        }

        Set neighborSet = map.entrySet();
        Iterator it = neighborSet.iterator();

        while(it.hasNext()) {
            if(k == 3) {
                break;
            }
            Map.Entry me = (Map.Entry) it.next();
            Point2D.Double neighbor = (Point2D.Double) me.getValue();
            // check to make sure the link doesn't collide with a wall, or that the link is between a node and itself
            if(linkCollidesWithWall(joint, neighbor) || (joint.equals(neighbor))) {
                continue;
            }
            neighbors.add(neighbor);
            k++;
        }
        return neighbors;
    }

    // Checks to make sure that a link connecting two points don't collide with a wall.
    public boolean linkCollidesWithWall(Point2D.Double joint, Point2D.Double point) {
        for(Rectangle wall : walls) {
            if(wall.intersectsLine(joint.x, joint.y, point.x, point.y)) {
                // it is being caught, but still drawing the lines...? - only when you translate the origin
//                System.out.println("Link Collided: " + point.x + ", " + point.y + " and " + joint.x +  ", " + joint.y);
                return true;
            }
        }
//        System.out.println("Didn't intersect!");
        return false;
    }

}
