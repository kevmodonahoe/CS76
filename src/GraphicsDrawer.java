import javax.swing.*;

/**
 * Created by kdonahoe on 10/2/16.
 */

public class GraphicsDrawer extends JFrame {

    public GraphicsDrawer(World robotWorld) {
        createGraphics(robotWorld);
    }

    public GraphicsDrawer(MobileRobotWorld mobileRobotWorld) {createMobileGraphics(mobileRobotWorld);}

    private void createMobileGraphics(MobileRobotWorld mobileRobotWorld) {
        add(mobileRobotWorld);

        setTitle("Mobile Robot Graphics");
        setSize(800, 800);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    private void createGraphics(World robotWorld) {
        add(robotWorld);

        setTitle("Robot Arm Graphics");
        setSize(800, 800);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

}
