package finallab;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.awt.Point;

import javax.swing.JPanel;

import org.openkinect.freenect.Device;

public abstract class KinectVideo extends JPanel {

	private static final long serialVersionUID = 1L;
	public final static int WIDTH = 640;
	public final static int HEIGHT = 480;
	public final static int C_X = WIDTH / 2;
	public final static int C_Y = HEIGHT / 2;
	public static double RGB_DEPTH_DIST = .025;
	
	protected BufferedImage frame;
	protected ByteBuffer frameData;
	protected boolean display;

	//calibration params
	protected double f;
	
	public KinectVideo(Device kinect, boolean _display) {
		setPreferredSize(new Dimension(WIDTH, HEIGHT));
		display = _display;
		if (display)
			frame = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
	}
	
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawImage(frame, 0, 0, null);           
    }
    
    public ByteBuffer getBuf() {
    	return frameData;
    }

    public BufferedImage getFrame(){
    	return frame;
    }
    public Point3D getWorldCoords(Point p, double depth) {
    	Point3D world = new Point3D();
		world.z = depth;
		world.x = (p.x) * (world.z / f);
		world.y = (p.y) * (world.z / f);
		return world;
    }
	public Point getPixFromWorld(Point3D world) {
		// world.x += RGB_DEPTH_DIST;
		Point pix = new Point();
		pix.x = (int)(f * world.x / world.z);
		pix.y = (int)(f * world.y / world.z);
		return pix;
	}
    public void pause() {
    	display = false;
    }
    public void resume() {
    	display = true;
    }
}
