import javax.swing.*;
import java.awt.*;

/**
 * Created by kdonahoe on 10/2/16.
 */
public class Wall extends JPanel {
    int x, y, size;
    public Wall(int initX, int initY, int initSize) {
        this.x = initX;
        this.y = initY;
        this.size = initSize;
    }

    private void doDrawing(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.drawLine(5, 5, 50, 5);
    }

    @Override
    public void paintComponent(Graphics g) {

        super.paintComponent(g);
        doDrawing(g);
    }

}
