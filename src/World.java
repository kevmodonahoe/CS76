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
    int baseX, baseY, armLength;
    GeneralPath robot;

    // A World is the actual area where all the arms and walls live.
    // An instance of this is instantiated inside of the RobotArmProblem.
    public World(ArrayList<Rectangle> walls, ArrayList<RobotArm> arms, GeneralPath robot) {
        ArrayList<Integer> angles = new ArrayList<>();
        this.walls = walls;
        this.arms = arms;
        this.robot = robot;
        this.baseX = 200;
        this.baseY = 550;
        this.armLength = 100;
        generateAngles(angles);
        this.joints = generateJoints(baseX, baseY, angles);

        generateWalls(walls);
        generateRobot(joints);

    }

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

    public void generateAngles(ArrayList<Integer> angles) {
        angles.add(320);
        angles.add(340);
        angles.add(280);
    }

    public ArrayList<Point2D.Double> generateJoints(int baseX, int baseY, ArrayList<Integer> angles) {
        ArrayList<Point2D.Double> joints = new ArrayList<Point2D.Double>();

        int firstAngle = angles.get(0);
        int secondAngle = angles.get(1);
        int thirdAngle = angles.get(2);

        Double x1_double = baseX + (armLength* Math.cos(Math.toRadians(firstAngle)));
        int x1 = x1_double.intValue();

        Double y1_double = baseY + (armLength * Math.sin(Math.toRadians(firstAngle)));
        int y1 = y1_double.intValue();


        Double x2_double = x1 + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle)));
        int x2 = x2_double.intValue();

        Double y2_double = y1 + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle)));
        int y2 = y2_double.intValue();

        Double x3_double = x2 + (armLength * Math.cos(Math.toRadians(firstAngle + secondAngle + thirdAngle)));
        int x3 = x3_double.intValue();

        Double y3_double = y2 + (armLength * Math.sin(Math.toRadians(firstAngle + secondAngle + thirdAngle)));
        int y3 = y3_double.intValue();

        // DO I NEED TO ADD THE INITIAL POINT TO THE JOINTS?
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

    public void generateRobot(ArrayList<Point2D.Double> joints) {
        int width = 100;
        int height = 1;
        ArrayList<Integer> angles = new ArrayList<>();
        angles.add(20);
        angles.add(340);
        angles.add(280);

//        for (int i=0; i<joints.size(); i++) {
//            arms.add(new RobotArm(width, height, angles.get(i), joints.get(i)));
//        }

//
//        int firstAngle = 0;
//
//        Point p1 = new Point(200, 550);
//        Double p1x_double = ((p1.x + width) * Math.cos(Math.toRadians(10)));
//        int p1x_int = p1x_double.intValue();
//
//
//        Double p2x_double = (p1x_int) + (width * Math.cos(Math.toRadians(10)));
//        int p2x_int = p2x_double.intValue();
//
//
//        Point p2 = new Point(p2x_int, p1.y);
//
//        // from the joints, draw connect the lines between them (create the arms!)
//        RobotArm robotArm1 = new RobotArm(width, height, firstAngle, p1);
//        RobotArm robotArm2 = new RobotArm(width, height, 335, p2);
//
////        Point p3 = new Point(p2.x + width, p2.y);
////        RobotArm robotArm3 = new RobotArm(width, height, 235, p3);
//
//        arms.add(robotArm1);
//        arms.add(robotArm2);
//        arms.add(robotArm3);

        // calls a getJoints function - the arms take in an angle, and the getJoints returns
        // the arraylist of lines (or robot arms) that make up the robot

        // then calls a function to generate the graph (successor function)
    }

    public ArrayList<Rectangle> getWalls() {
        return walls;
    }

    public ArrayList<RobotArm> getArms() {
        return arms;
    }

    // Actually draws the world, which includes all the walls and arms.
    private void drawWorld(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;

        // actually draws the walls in the world
        for(Rectangle wall : walls) {
            g2d.fill(wall);
        }
//
//        for(RobotArm arm : arms) {
//            g2d.draw(arm.getArm());
//        }


        robot.moveTo(joints.get(0).x, joints.get(0).y);
        for(int i=1; i<joints.size(); i++) {
            robot.lineTo(joints.get(i).x, joints.get(i).y);
        }
        g2d.draw(robot);

    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawWorld(g);
    }
}
