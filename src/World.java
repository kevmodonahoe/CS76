import javax.swing.*;

import java.awt.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import javax.swing.JFrame;
import javax.swing.JPanel;

import static java.awt.BasicStroke.CAP_ROUND;

/**
 * Created by kdonahoe on 10/2/16.
 */

class World extends JPanel {
    ArrayList<Rectangle> walls;
    ArrayList<RobotArm> arms;
    ArrayList<Point2D.Double> joints;
    ArrayList<Point> samplePoints;
    int[] startConfig;
    int[] goalConfig;
    int baseX, baseY, armLength;
    GeneralPath robot;

    // A World is the actual area where all the arms and walls live.
    // An instance of this is instantiated inside of the RobotArmProblem.
    public World(ArrayList<Rectangle> walls, ArrayList<RobotArm> arms, GeneralPath robot, int numJoints) {
        // The state of the problem is the angles
        samplePoints = new ArrayList<>();
        this.walls = walls;
        this.arms = arms;
        this.robot = robot;
        this.baseX = 0;
        this.baseY = 0;
        this.armLength = 100;
        ArrayList<Integer> startAngles = generateStartAngles();
        ArrayList<Integer> goalAngles = generateGoalAngles();
        this.startConfig = generateStartConfig(numJoints, startAngles);
        this.goalConfig = generateGoalConfig(numJoints, goalAngles);
        this.joints = generateJoints(baseX, baseY, startAngles);
        generateWalls(walls);

    }

    // Generates the walls that act as obsticals for the robot.
    public void generateWalls(ArrayList<Rectangle> walls) {
        Rectangle wall1 = new Rectangle(-200, -200, 100, 100);
        Rectangle wall2 = new Rectangle(100, -200, 100, 100);
        Rectangle wall3 = new Rectangle(-200, 100, 100, 100);
        Rectangle wall4 = new Rectangle(100, 100, 100, 100);
        walls.add(wall1);
        walls.add(wall2);
        walls.add(wall3);
        walls.add(wall4);
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

    // Generates the starting config for the robot.
    // This int array's first two indices are the robot's base coordinates (0, 0),
    // and the remaining indices are the angles of the remaining coordinates.
    public int[] generateStartConfig(int numJoints, ArrayList<Integer> angles) {
        int[] startConfig = new int[numJoints + 2];
        startConfig[0] = 0;
        startConfig[1] = 0;
        for(int i=0; i<numJoints; i++) {
            startConfig[i + 2] = angles.get(i);
        }

        return startConfig;
    }

    // Generates the goal config for the robot.
    // This int array's first two indices are the goal robot's base coordinates (0, 0),
    // and the remaining indices are the angles of the remaining coordinates.
    public int[] generateGoalConfig(int numJoints, ArrayList<Integer> angles) {
        int[] goalConfig = new int[numJoints + 2];
        goalConfig[0] = 0;
        goalConfig[1] = 0;
        for(int i=0; i<numJoints; i++) {
            goalConfig[i + 2] = angles.get(i);
        }

        return goalConfig;
    }

    // Given the angles of the links of arms, this function generates the endpoints where the joints
    // will be represented.
    // It returns an ArrayList containing the location of each joint.
    public ArrayList<Point2D.Double> generateJoints(int baseX, int baseY, ArrayList<Integer> angles) {
        ArrayList<Point2D.Double> joints = new ArrayList<Point2D.Double>();

        int firstAngle = angles.get(0);
        int secondAngle = angles.get(1);
        int thirdAngle = angles.get(2);

        Double x1_double = baseX + (armLength* Math.cos(Math.toRadians(firstAngle)));
        Double y1_double = baseY + (armLength * Math.sin(Math.toRadians(firstAngle)));
        Double x2_double = x1_double + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle)));
        Double y2_double = y1_double + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle)));
        Double x3_double = x2_double + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle + thirdAngle)));
        Double y3_double = y2_double + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle + thirdAngle)));

        Point2D.Double initialPoint = new Point2D.Double(baseX, baseY);
        Point2D.Double p1 = new Point2D.Double(x1_double, y1_double);
        Point2D.Double p2 = new Point2D.Double(x2_double, y2_double);
        Point2D.Double p3 = new Point2D.Double(x3_double, y3_double);

        joints.add(initialPoint);
        joints.add(p1);
        joints.add(p2);
        joints.add(p3);
        return joints;
    }

    // Checks to see if the current config is the goal config.
    public boolean goalState(int[] config) {
       return config == goalConfig;
    }

    public ArrayList<Rectangle> getWalls() {
        return walls;
    }

    public ArrayList<RobotArm> getArms() {
        return arms;
    }

    public GeneralPath getRobot() { return robot; }

    public ArrayList<Point> getSamplePoints() { return samplePoints; }

    // Actually draws the world, which includes all the walls and arms.
    private void drawWorld(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;

        // actually draws the walls in the world
        for(Rectangle wall : walls) {
            g2d.draw(wall);
        }

        robot.moveTo(joints.get(0).x, joints.get(0).y);
        for(int i=1; i<joints.size(); i++) {
            g2d.fillOval((int) joints.get(i).x, (int) joints.get(i).y, 10, 10);
            robot.lineTo(joints.get(i).x, joints.get(i).y);
        }

        g2d.draw(robot);
    }

    public void updateWorldWithSampleNodes(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        if(samplePoints.size() > 0) {
            for(Point point : samplePoints) {
                g2d.drawOval(point.x, point.y, 5, 5);
            }
        }
        repaint();

    }

    public void updateWorldWithKNeighbors(Graphics g, Point2D.Double joint, ArrayList<Point> neighbors) {
        Graphics2D g2d = (Graphics2D) g;
        for(Point point : neighbors) {
            robot.moveTo(joint.x, joint.y);
            robot.lineTo(point.x, point.y);
        }
        g2d.draw(robot);
        repaint();
    }


    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.translate(400, 400);
        g.drawLine(0, -400, 0, 400);
        g.drawLine(-400, 0, 400, 0);
        drawWorld(g);
    }
}
