/**
 * Created by kdonahoe on 10/2/16.
 */

import java.awt.Point;
import java.awt.Shape;
import java.awt.Rectangle;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class RobotArm {
    private int width, height;
    private float angle;
    Point2D.Double point;

    public RobotArm(int width, int height, float angle, Point2D.Double point) {
        this.width = width;
        this.height = height;
        this.angle = angle;
        this.point = point;
    }

    // Rotates the arm by "angle" degrees and returns the Arm as a Shape object
    public Shape getArm() {
        Shape shape = AffineTransform.getRotateInstance(Math.toRadians(angle), point.x, point.y).createTransformedShape(new Rectangle(((int) point.x), (int) point.y, width, height));
        return shape;
    }

    // get joints function

    // Checks to see if the Robot Arm intersects with any of the walls in the world
    public boolean doesIntersect(ArrayList<Rectangle> walls) {
        for(Rectangle wall : walls) {
            if(this.getArm().intersects(wall)) {
                return true;
            }
        }
        return false;
    }
}
