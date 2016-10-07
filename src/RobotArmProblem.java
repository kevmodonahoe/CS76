import java.awt.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.util.ArrayList;

/**
 * Created by kdonahoe on 10/2/16.
 */
public class RobotArmProblem {
    static ArrayList<Rectangle> walls = new ArrayList<Rectangle>();
    static ArrayList<RobotArm> arms = new ArrayList<RobotArm>();
    static GeneralPath robot = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);

    public RobotArmProblem(ArrayList<Rectangle> obsticles, ArrayList<RobotArm> robotArms) {
        this.walls = obsticles;
        this.arms = robotArms;
    }


    public boolean goalState() {
        return false;
    }


    public static void main(String[] args) {
        World robotWorld = new World(walls, arms, robot, 3);
        GraphicsDrawer drawer = new GraphicsDrawer(robotWorld);
        drawer.setVisible(true);

        // Need to sleep first so that all the variables of robotWorld are set before the rest of the main runs.
        // Otherwise, the robot would have no points/paths.
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        walls = robotWorld.getWalls();
        arms = robotWorld.getArms();
        robot = robotWorld.getRobot();

        for(Rectangle wall : walls) {
            if (robot.intersects(wall)) {
                System.out.println("Intersects!!");
                robotWorld.setBackground(Color.red);
            } else {
                System.out.println("Doesn't intersect...");
            }
        }

        /* Now that the world has been created, let's start the Probabilistic Road Mapping
            Three steps will be involved for PRM:
            1) Sampling method - create vertices of the graph in the configuration space
            2) Collision detection method - check to see if these vertices are legal
            3) Local planner - attempts to connect vertices
        */
        PRM prm = new PRM(robotWorld);
        prm.performPRM();



        // run the program, and pass the world into it!

        System.out.println("Graphics ran!!");

    }
}
