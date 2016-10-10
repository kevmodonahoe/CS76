import java.awt.*;
import java.awt.geom.GeneralPath;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.awt.geom.Ellipse2D;

/**
 * Created by kdonahoe on 10/2/16.
 */
public class RobotArmProblem {
    static ArrayList<Rectangle> rectWalls = new ArrayList<>();
    static ArrayList<Ellipse2D> circularWalls = new ArrayList<>();
    static ArrayList<RobotArm> arms = new ArrayList<RobotArm>();
    static GeneralPath robot = new GeneralPath(GeneralPath.WIND_EVEN_ODD, 100);


    public RobotArmProblem(ArrayList<Rectangle> obsticles, ArrayList<RobotArm> robotArms) {
        this.rectWalls = obsticles;
        this.arms = robotArms;
    }


    public boolean goalState() {
        return false;
    }


    public static void main(String[] args) throws IOException {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.println("Which program would you like to run? \n1 for Robot Arm Standard \n2 for Robot Arm " +
                "Difficult \n3 for Mobile Robot \nYour choice: ");
        String userInput = br.readLine();
        int input = Integer.parseInt(userInput);
        World robotWorld;

        if(input == 1) {
            robotWorld = new World(rectWalls, circularWalls, arms, robot, 3, false);
            GraphicsDrawer drawer = new GraphicsDrawer(robotWorld);
            drawer.setVisible(true);

            // Need to sleep first so that all the variables of robotWorld are set before the rest of the main runs.
            // Otherwise, the robot would have no points/paths.
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        /* Now that the world has been created, let's start the Probabilistic Road Mapping
            Three steps will be involved for PRM:
            1) Sampling method - create vertices of the graph in the configuration space
            2) Collision detection method - check to see if these vertices are legal
            3) Local planner - attempts to connect vertices
        */
            PRM prm = new PRM(robotWorld);
            prm.performPRM();

        } else if( input == 2) {
            robotWorld = new World(rectWalls, circularWalls, arms, robot, 3, true);
            GraphicsDrawer drawer = new GraphicsDrawer(robotWorld);
            drawer.setVisible(true);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            PRM prm = new PRM(robotWorld);
            prm.performPRM();


        } else if (input == 3) {
            System.out.println("Running mobile");
            MobileRobotWorld mobileRobotWorld = new MobileRobotWorld();
            GraphicsDrawer drawer = new GraphicsDrawer(mobileRobotWorld);
            drawer.setVisible(true);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            RRT rrt = new RRT(mobileRobotWorld);
            rrt.performRRT();

        } else {
            System.out.println("Please enter number between 1-3.");
            return;
        }



        System.out.println("Graphics ran!!");

    }
}
