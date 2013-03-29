package finallab;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JPanel;

import org.openkinect.freenect.Device;

public abstract class KinectVideo extends JPanel {

	private static final long serialVersionUID = 1L;
	public final static int WIDTH = 640;
	public final static int HEIGHT = 480;
	
	protected BufferedImage frame;
	
	public KinectVideo(Device kinect) {
		setPreferredSize(new Dimension(WIDTH, HEIGHT));
		frame = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
	}
	
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawImage(frame, 0, 0, null);           
    }

}
