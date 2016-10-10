import com.sun.tools.javac.jvm.Gen;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Created by kdonahoe on 10/10/16.
 */
public class MobileRobotWorld extends JPanel {
//    GeneralPath robot;
    GeneralPath map;
    MobileRobot robot;

    public MobileRobotWorld() {
        map = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);
        Point2D.Double robotPos = new Point2D.Double(400, 400);
        robot = new MobileRobot(robotPos, 90, 0, 0);
    }

    public void updateWorldWithSampleNodes(Graphics g, ArrayList<Point2D.Double> samplePoints) {
        Graphics2D g2d = (Graphics2D) g;
        if(samplePoints.size() > 0) {
            for(Point2D.Double point : samplePoints) {
                g2d.fillOval((int) point.x, (int) point.y, 5, 5);
            }
        }
    }

    public void updateWorldWithLink(Graphics g, Point2D.Double point1, Point2D.Double point2) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.drawLine((int) point1.x, (int) point1.y, (int) point2.x, (int) point2.y);
    }

    public void drawWorld(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;
        g2.fillRoundRect((int) robot.position.x, (int) robot.position.y, 20, 40, 10, 10);
        g2.setStroke(new BasicStroke(3));
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawLine(0, 400, 400, 400);
        drawWorld(g);
    }
}
