import org.w3c.dom.css.Rect;

import javax.swing.*;

import java.awt.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import javax.swing.JFrame;
import javax.swing.JPanel;
import java.awt.geom.Ellipse2D;

import static java.awt.BasicStroke.CAP_ROUND;

/**
 * Created by kdonahoe on 10/2/16.
 */

class World extends JPanel {
    ArrayList<Rectangle> rectWalls;
    ArrayList<Ellipse2D> circularWalls;
    ArrayList<RobotArm> arms;
    ArrayList<Point2D.Double> joints;
    ArrayList<Point2D.Double> goalJoints;
    Point2D.Double basePoint;

    State startingConfig;
    State goalConfig;
    int armLength;
    GeneralPath robot;
    GeneralPath goalPath;

    // A World is the actual area where all the arms and walls live.
    // An instance of this is instantiated inside of the RobotArmProblem.
    public World(ArrayList<Rectangle> rectWalls, ArrayList<Ellipse2D> circularWalls, ArrayList<RobotArm> arms, GeneralPath robot, int numJoints) {
        // The state of the problem is the angles
        goalPath = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);
        this.basePoint = new Point2D.Double(400, 400);
        this.rectWalls = rectWalls;
        this.circularWalls = circularWalls;
        this.arms = arms;
        this.robot = robot;
        this.armLength = 90;

        ArrayList<Integer> startAngles = generateStartAngles();
        ArrayList<Integer> goalAngles = generateGoalAngles();
        this.startingConfig = new State();
        this.goalConfig = new State();
        this.startingConfig.angles.addAll(startAngles);
        this.goalConfig.angles.addAll(goalAngles);

        this.joints = generateJoints(basePoint, startAngles);
        this.goalJoints = generateJoints(basePoint, goalAngles);
        generateWalls(rectWalls, circularWalls);

    }

    // Generates the walls that act as obsticals for the robot.
    public void generateWalls(ArrayList<Rectangle> rectWalls, ArrayList<Ellipse2D> circularWalls) {
        Rectangle wall1 = new Rectangle(200, 200, 100, 100);
        Rectangle wall2 = new Rectangle(500, 200, 100, 100);
        Rectangle wall3 = new Rectangle(200, 500, 100, 100);
        Rectangle wall4 = new Rectangle(500, 500, 100, 100);

        Ellipse2D wall5 = new Ellipse2D.Double(400, 300, 50, 50);
        Ellipse2D wall6 = new Ellipse2D.Double(300, 450, 25, 25);

        rectWalls.add(wall1);
        rectWalls.add(wall2);
        rectWalls.add(wall3);
        rectWalls.add(wall4);
//        circularWalls.add(wall5);
//        circularWalls.add(wall6);
    }

    // Generates the initial angles to be used for the starting robot position.
    public ArrayList<Integer> generateStartAngles() {
        ArrayList<Integer> startAngles = new ArrayList<>();
        startAngles.add(0);
        startAngles.add(0);
        startAngles.add(270);
        return startAngles;
    }

    // Generates the goal angles used for the goal robot position.
    public ArrayList<Integer> generateGoalAngles() {
        ArrayList<Integer> goalAngles = new ArrayList<>();
        goalAngles.add(180);
        goalAngles.add(0);
        goalAngles.add(270);
        return goalAngles;
    }

    // Given the angles of the links of arms, this function generates the endpoints where the joints
    // will be represented.
    // It returns an ArrayList containing the location of each joint.
    public ArrayList<Point2D.Double> generateJoints(Point2D.Double basePoint, ArrayList<Integer> angles) {
        ArrayList<Point2D.Double> joints = new ArrayList<Point2D.Double>();

        int firstAngle = angles.get(0);
        int secondAngle = angles.get(1);
        int thirdAngle = angles.get(2);

        Double x1_double = basePoint.x + (armLength* Math.cos(Math.toRadians(firstAngle)));
        Double y1_double = basePoint.y + (armLength * Math.sin(Math.toRadians(firstAngle)));
        Double x2_double = x1_double + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle)));
        Double y2_double = y1_double + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle)));
        Double x3_double = x2_double + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle + thirdAngle)));
        Double y3_double = y2_double + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle + thirdAngle)));

        Point2D.Double initialPoint = new Point2D.Double(basePoint.x, basePoint.y);
        Point2D.Double p1 = new Point2D.Double(x1_double, y1_double);
        Point2D.Double p2 = new Point2D.Double(x2_double, y2_double);
        Point2D.Double p3 = new Point2D.Double(x3_double, y3_double);

        joints.add(initialPoint);
        joints.add(p1);
        joints.add(p2);
        joints.add(p3);
        return joints;
    }

    public ArrayList<Rectangle> getRectWalls() {
        return rectWalls;
    }

    public ArrayList<RobotArm> getArms() {
        return arms;
    }

    public GeneralPath getRobot() { return robot; }

    // Actually draws the world, which includes all the walls and arms.
    private void drawWorld(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;

        GeneralPath startRobot = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);
        GeneralPath goalRobot = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);

        // actually draws the walls in the world
        for(Rectangle wall : rectWalls) {
            g2.fill(wall);
        }

        for (Ellipse2D circularWall : circularWalls) {
            g2.fill(circularWall);
        }

        // draw the start configuration
        startRobot.moveTo(joints.get(0).x, joints.get(0).y);
        for(int i=1; i<joints.size(); i++) {
            g2.fillOval((int) joints.get(i).x, (int) joints.get(i).y, 10, 10);
            startRobot.lineTo(joints.get(i).x, joints.get(i).y);
        }

        // draw the goal configuration
        goalRobot.moveTo(goalJoints.get(0).x, goalJoints.get(0).y);
        for(int j=1; j<goalJoints.size(); j++) {
            g2.fillOval((int) goalJoints.get(j).x, (int) goalJoints.get(j).y, 10, 10);
            goalRobot.lineTo(goalJoints.get(j).x, goalJoints.get(j).y);

        }
        g2.setStroke(new BasicStroke(3));
        g2.setPaint(Color.green);
        g2.draw(startRobot);
        g2.setPaint(Color.red);
        g2.draw(goalRobot);
        g2.setPaint(Color.black);

    }

    public void updateWorldWithSampleNodes(Graphics g, ArrayList<Point2D.Double> samplePoints) {
        Graphics2D g2d = (Graphics2D) g;
        if(samplePoints.size() > 0) {
            for(Point2D.Double point : samplePoints) {
                g2d.fillOval((int) point.x, (int) point.y, 5, 5);
            }
        }
    }

    public void updateWorldConnectSampleNodes(Graphics g, ArrayList<Point2D.Double> samplePoints) {
        Graphics2D g2d = (Graphics2D) g;
        robot.moveTo(400, 400);
        for(Point2D.Double point : samplePoints) {
            robot.lineTo(point.x, point.y);
            robot.moveTo(point.x, point.y);
        }
        g2d.draw(robot);
    }

    public void updateWorldWithKNeighbors(Graphics g, Point2D.Double joint, ArrayList<Point2D.Double> neighbors) {
        Graphics2D g2d = (Graphics2D) g;
        for(Point2D.Double point : neighbors) {
            robot.moveTo(joint.x, joint.y);
            robot.lineTo(point.x, point.y);
        }
        g2d.draw(robot);
    }

    public void updateWorldFinalPath(ArrayList<Point2D.Double> finalPath, Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setStroke(new BasicStroke(3));
        for(int i=0; i<finalPath.size() - 1; i++) {
            goalPath.moveTo(finalPath.get(i).x, finalPath.get(i).y);
            goalPath.lineTo(finalPath.get(i + 1).x, finalPath.get(i + 1).y);
        }
        g2d.draw(goalPath);
    }


    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawWorld(g);
    }
}
