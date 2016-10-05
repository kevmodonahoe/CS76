/**
 * Created by kdonahoe on 10/5/16.
 */

import com.sun.corba.se.impl.orbutil.graph.Graph;
import org.w3c.dom.css.Rect;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import java.awt.Point;
import java.util.Set;
import java.util.HashSet;
import java.util.Random;

public class PRM {
    ArrayList<Rectangle> walls;
    ArrayList<Point> samplePoints;
    GeneralPath robot;
    World robotWorld;

    public PRM(World robotWorld) {
        this.robotWorld = robotWorld;
        this.robot = robotWorld.getRobot();
        this.walls = robotWorld.getWalls();
        this.samplePoints = robotWorld.getSamplePoints();

    }

    public void performPRM() {
        samplePoints = generateSamplePoints();
        System.out.println("sample size: " + samplePoints.size());
        System.out.println("updating graphics");
        robotWorld.updateWorld(robotWorld.getGraphics());

    }

    // sampling method
    public ArrayList<Point> generateSamplePoints() {
        Random rand = new Random();
        Point point;

        while(samplePoints.size() < 10) {
            point = new Point();
            point.x = rand.nextInt(600) + 1;
            point.y = rand.nextInt(600) + 1;

            // Makes sure that there are no collisions with walls, other points, or duplicate points.
            if(isCollision(point) || closeToOtherPoint(point)) {
                System.out.println("Collided or was too close to another point: " + point.x  + ", " + point.y);
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

    // collision detection method
    public boolean isCollision(Point point) {
        ArrayList<Rectangle> walls = robotWorld.getWalls();
        for(Rectangle wall : walls) {
            if((point.x > wall.getX() && point.x <= (wall.getX() + wall.getWidth())) &&
                    (point.y > wall.getY() && point.y <= (wall.getY() + wall.getHeight()))) {
                return true;
            }
        }
        return false;
    }

    // local planner

}
