import org.w3c.dom.css.Rect;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

/**
 * Created by kdonahoe on 10/2/16.
 */

public class GraphicsDrawer extends JFrame {

    public GraphicsDrawer(World robotWorld) {
        createGraphics(robotWorld);
    }

    private void createGraphics(World robotWorld) {
        add(robotWorld);

        setTitle("Robot Graphics");
        setSize(700, 700);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }
}
