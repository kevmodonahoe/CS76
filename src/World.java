import javax.swing.*;

import java.awt.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 * Created by kdonahoe on 10/2/16.
 */

class World extends JPanel {
    ArrayList<Rectangle> walls;
    ArrayList<RobotArm> arms;
    ArrayList<Point2D.Double> joints;
    ArrayList<Point> samplePoints;
    int baseX, baseY, armLength;
    GeneralPath robot;

    // A World is the actual area where all the arms and walls live.
    // An instance of this is instantiated inside of the RobotArmProblem.
    public World(ArrayList<Rectangle> walls, ArrayList<RobotArm> arms, GeneralPath robot) {
        // The state of the problem is the angles
        ArrayList<Integer> angles = new ArrayList<>();
        samplePoints = new ArrayList<>();
        this.walls = walls;
        this.arms = arms;
        this.robot = robot;
        this.baseX = 200;
        this.baseY = 550;
        this.armLength = 100;
        generateAngles(angles);
        this.joints = generateJoints(baseX, baseY, angles);
        generateWalls(walls);

    }

    // Generates the walls that act as obsticals for the robot.
    public void generateWalls(ArrayList<Rectangle> walls) {
        Rectangle wall1 = new Rectangle(200, 150, 100, 100);
        Rectangle wall2 = new Rectangle(400, 150, 100, 100);
        Rectangle wall3 = new Rectangle(200, 300, 100, 100);
        Rectangle wall4 = new Rectangle(400, 300, 100, 100);
        walls.add(wall1);
        walls.add(wall2);
        walls.add(wall3);
        walls.add(wall4);
    }

    // Generates the initial angles to be used for the starting robot position.
    public void generateAngles(ArrayList<Integer> angles) {
        angles.add(0);
        angles.add(340);
        angles.add(280);
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
            g2d.fill(wall);
        }

        robot.moveTo(joints.get(0).x, joints.get(0).y);
        for(int i=1; i<joints.size(); i++) {
            System.out.println("Joints: " + joints.get(i).x + " " + joints.get(i).y);
            g2d.drawOval((int) joints.get(i).x, (int) joints.get(i).y, 5, 5);
            robot.lineTo(joints.get(i).x, joints.get(i).y);
        }
        g2d.draw(robot);
    }

    public void updateWorld(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        if(samplePoints.size() > 0) {
            for(Point point : samplePoints) {
                g2d.drawOval(point.x, point.y, 10, 10);
            }
        }
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawWorld(g);
        updateWorld(g);
    }
}
