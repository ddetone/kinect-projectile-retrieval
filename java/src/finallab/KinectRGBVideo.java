package finallab;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import javax.swing.JPanel;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.FrameMode;
import org.openkinect.freenect.VideoHandler;

public class KinectRGBVideo extends JPanel {
	//from JPanel
	private static final long serialVersionUID = 1L;
	
	public final static int WIDTH = 640;
	public final static int HEIGHT = 480;
	
	private BufferedImage frame;
	
	public KinectRGBVideo(Device kinect) {
		
		frame = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
		
		kinect.startVideo(new VideoHandler() {
			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int timestamp) {
				int[] pixelInts = new int[WIDTH * HEIGHT];

				for(int i = 0; i < WIDTH*HEIGHT; i++) {
					int rgbVal = 0xFF;
					for(int j = 0; j < 3; j++) {
						int data = rgb.get() & 0xFF;
						rgbVal = rgbVal << 8;
						rgbVal = rgbVal | (data);
					}
					pixelInts[i] = rgbVal;
				}
				
				frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
				rgb.position(0);
			}
			
		});
	}
	
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawImage(frame, 0, 0, null);           
    }

	
}
