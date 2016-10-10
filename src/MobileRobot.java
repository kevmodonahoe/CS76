import java.awt.geom.Point2D;

/**
 * Created by kdonahoe on 10/10/16.
 */
public class MobileRobot {
    Point2D.Double position;
    int angle;
    int v; // Vertical Velocity
    int w; // Angular Velocity

    public MobileRobot(Point2D.Double position, int angle, int v, int w) {
        this.position = position;
        this.angle = angle;
        this.v = v;
        this.w = w;
    }
}
