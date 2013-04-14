package finallab;

import java.nio.ByteBuffer;
import java.awt.Point;
import java.util.*;

import org.openkinect.freenect.DepthFormat;
import org.openkinect.freenect.DepthHandler;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.FrameMode;


import april.util.*;
import javax.swing.*;

public class KinectDepthVideo extends KinectVideo {
	//from JPanel
	private static final long serialVersionUID = 2;	
	
	public volatile boolean newImage = false;

	private final static int MAX_FRAMES = 100;
	private final static int THRESH = 50;

	private double [] depthAvgs;
	private short [] validPixels;
	private int numFrames;
	
	public KinectDepthVideo(Device kinect, boolean _display) {
		super(kinect, _display);	

		numFrames = 0;
		depthAvgs = new double[WIDTH*HEIGHT];
		validPixels = new short[WIDTH*HEIGHT];
		for (int i = 0; i < WIDTH*HEIGHT; i++) {
			depthAvgs[i] = 0;
			validPixels[i] = -1;
			depthAvgs[i] = 2047.0;
		}

		f = 585.124;
		final ParameterGUI pg = new ParameterGUI();
		pg.addDoubleSlider("thresh", "thresh", 1, 100, 50);
		pg.addIntSlider("frames", "frames", 1, 1000, 200);
		pg.addDoubleSlider("learning", "learning", 0.0, 1.0, .05);
		JFrame slider = new JFrame("thresh slider");

		slider.setSize(300, 100);
		slider.add(pg);
		slider.setVisible(true);

		kinect.setDepthFormat(DepthFormat.D11BIT);
		kinect.startDepth(new DepthHandler() {

		
			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer depthBuf, int _timestamp) {

				frameData = depthBuf;
				timestamp = _timestamp;
				int [] pixelInts = null;
			
				if (display)
					pixelInts = new int[WIDTH * HEIGHT];
				int[] avgInts = new int[WIDTH*HEIGHT];

				//ballDepth = getDepth(depthBuf,BALL.center_x*width + BALL.center_y);

				frameLock.writeLock().lock();
				try {
					for(int i = 0; i < WIDTH*HEIGHT; i++) {
						int depth = 0;
						byte byte1;
						byte byte2;
						try{
							byte1 = depthBuf.get();
							byte2 = depthBuf.get();
						}
						catch(Exception e)
						{
							System.out.println("depth recieveing error");
							depthBuf.position(0);
							return;
						}
						depth = byte2 & 0x7;
						depth = depth << 8;
						depth = depth | (byte1 & 0xFF);

						//background subtraction
						validPixels[i] = -1;
						boolean valid = false;
						if(depth < 1050)
						{
							if (/*depth<1000 &&*/ (depthAvgs[i] - (double)depth) > pg.gd("thresh")) {
								valid = true;
								validPixels[i] = (short)depth;			
							}
							//double depthFactor = (((depthAvgs[i] * (1.0-pg.gd("learning")) * (double)numFrames) + (pg.gd("learning")*(double)depth)) / (double)(numFrames + 1));
							depthAvgs[i] = (((depthAvgs[i] * (double)numFrames) + (double)depth) / (double)(numFrames + 1));
						}




						if (display) {

							/*
							 * color scaled depth
							 * closest -> farthest
							 * black -> red -> yellow -> green
							 * (depth starts registering ~390 or ~ .5m)
							 */
							int r = 0x0000;
							int g = 0x0000;
							int b = 0x0000;
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
							try{

							if(valid)
								pixelInts[i] = depthColor;
							else
								pixelInts[i] = 0xFF000000;
							}
							catch(Exception e)
							{
								System.out.println("pixelInts null errror");
								depthBuf.position(0);
								return;
							}
						}
					
					}
					depthBuf.position(0);
					newImage = true;
					if (display) {
						frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
						//if(numFrames >= pg.gi("frames"))
							//numFrames--;
						numFrames = (numFrames + 1) % pg.gi("frames");
						//numFrames++;
						//set position to 0 because ByteBuffer is reused to access byte array of new frame
						//and get() below increments the iterator
						// repaint();
						
					}
				}
				finally{
					frameLock.writeLock().unlock();
				}
			}			
		});
	}	
	// 0,0 at top left
	public float getDepthFromDepthPixel(Point p) {

		return raw_depth_to_meters(getDepthValFromDepthPixel(p) & 0x7FF);
	}
	public int getDepthValFromDepthPixel(Point p) {
		int index = p.y*WIDTH + p.x;
		int depth = 0;
		// try {
		byte byte1;
		byte byte2;
		try {
			byte1 = frameData.get(index * 2);
			byte2 = frameData.get(index * 2 + 1);
		} catch(Exception e) {
			System.out.println("can't get depth at " + p.x + "," + p.y);
			return 0;
		}
		depth = byte2 & 0x7;
		depth = depth << 8;
		depth = depth | (byte1 & 0xFF);
		return depth;
	}

	public float getDepthFromRGBPixel(Point poi, int cap) {
		return getDepthFromRGBPixel(poi, 160, cap);
	}

	public float getDepthFromRGBPixel(Point poi, int bound, int cap) {
		BallTracker bt = new BallTracker(WIDTH, HEIGHT, false);
		ArrayList<Statistics> ballStats = bt.analyzeDepthPartition(frameData, poi, bound, cap);
		return raw_depth_to_meters(ballStats.get(0).closestDepth);

	}

	public Point3D getWorldCoords(Point p) {
		Point pPix = new Point();
		pPix.x = p.x + C_X;
		pPix.y = C_Y - p.y;
		double depth = (double)getDepthFromDepthPixel(p);
		return getWorldCoords(pPix, depth);
	}

	public short [] getValidImageArray() {
		return validPixels;
	}
	public float raw_depth_to_meters(int raw_depth)
	{
 		if (raw_depth < 2047)
  		{
   			return (1.0f / (raw_depth * -0.0030711016f + 3.3309495161f))+.05f;
  		}
  		return 0;
	}


}
