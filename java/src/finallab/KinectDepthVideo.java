package finallab;

import java.nio.ByteBuffer;

import org.openkinect.freenect.DepthFormat;
import org.openkinect.freenect.DepthHandler;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.FrameMode;

public class KinectDepthVideo extends KinectVideo {
	//from JPanel
	private static final long serialVersionUID = 2;	
	
	public KinectDepthVideo(Device kinect) {
		super(kinect);	
		kinect.setDepthFormat(DepthFormat.D11BIT);
		kinect.startDepth(new DepthHandler() {

			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer depthBuf, int timestamp) {
				int[] pixelInts = new int[WIDTH * HEIGHT];

				//ballDepth = getDepth(depthBuf,BALL.center_x*width + BALL.center_y);

				for(int i = 0; i < WIDTH*HEIGHT; i++) {
					
					int depth = 0;
					byte byte1 = depthBuf.get();
					byte byte2 = depthBuf.get();
					depth = byte2 & 0x7;
					depth = depth << 8;
					depth = depth | (byte1 & 0xFF);

					/*
					 * color scaled depth
					 * closest -> farthest
					 * black -> red -> yellow -> green
					 * (depth starts registering ~390 or ~ .5m)
					 */
					int r = 0x0000;
					int g = 0x0000;
					int b = 0x0000;
					if (i == ((HEIGHT /2) * WIDTH + WIDTH/2)) {
					 	System.out.println("depth: "+ depth);

					}
					//square the depth because it grows much smaller at longer distances
					depth = depth * depth;
					int interval = (int)Math.pow(1100, 2d) / 6;
					double step = 255.0 / (double)interval;
					if (depth <= (interval)) {	
						//black->red
						r = (int)(depth * step) & 0xFF;
					}
					else if (depth <= (2*interval)) {
						//red->yellow
						r = 0xFF;
						g = (int)((depth - interval) * step) & 0xFF;
					}
					else if (depth <= (3*interval)) {
						//yellow->green
						r = (int)((interval - (depth % interval)) * step) & 0xFF;
						g = 0xFF;
					}
					else if (depth <= (4*interval)){
						//green->teal
						g = 0xFF;
						b = (int)((depth % interval) * step) & 0xFF;
					}
					else if (depth <= (5*interval)){
						//teal->blue
						g = (int)((interval - (depth % interval)) * step) & 0xFF;
						b = 0xFF;
					}
					else if (depth <= (6*interval)){
						//blue->purple
						r = (int)((depth % interval) * step) & 0xFF;
						b = 0xFF;
					}
					else {
						
					}
							
					int depthColor = 0xFF;			
					depthColor = depthColor << 8;
					depthColor = depthColor | (r & 0xFF);
					depthColor = depthColor << 8;
					depthColor = depthColor | (g & 0xFF);
					depthColor = depthColor << 8;
					depthColor = depthColor | (b & 0xFF);
					pixelInts[i] = depthColor;
				}
				frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
				
				//set position to 0 because ByteBuffer is reused to access byte array of new frame
				//and get() below increments the iterator
				depthBuf.position(0);
				repaint();
			}			
		});
	}	
	
}
