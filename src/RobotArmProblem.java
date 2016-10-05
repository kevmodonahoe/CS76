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
        World robotWorld = new World(walls, arms, robot);
        GraphicsDrawer drawer = new GraphicsDrawer(robotWorld);
        drawer.setVisible(true);

        walls = robotWorld.getWalls();
        arms = robotWorld.getArms();
        robot = robotWorld.robot;

        for(Rectangle wall : walls) {
            if (robot.intersects(wall)) {
                System.out.println("Intersects!!");
                robotWorld.setBackground(Color.red);
            } else {
                System.out.println("Doesn't intersect...");
                robotWorld.setBackground(Color.white);
            }
        }

//        for(RobotArm arm : arms) {
//            if(arm.doesIntersect(walls)) {
//
//        }


        // run the program, and pass the world into it!

        System.out.println("Graphics ran!!");

    }
}
