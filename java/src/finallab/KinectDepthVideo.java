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
	
	public Object gotImage;

	public static int MAX_FRAMES = 100;
	public static int THRESH = 50;
	public static int MAX_DEPTH = 1100;

	private double [] depthAvgs;
	private short [] validPixels;
	private int [] switchCount;
	private int numFrames;
	public Point botLoc;
	
	public long latestTimestamp;

	public ArrayList<Statistics> trajectory;

	private volatile boolean showAll = true;
	
	public KinectDepthVideo(Device kinect, Object _imgMonitor, boolean _display) {
		super(kinect, _imgMonitor, _display);	
		gotImage = new Object();
		numFrames = 0;
		depthAvgs = new double[WIDTH*HEIGHT];
		validPixels = new short[WIDTH*HEIGHT];
		trajectory = new ArrayList<Statistics>();
		switchCount = new int[WIDTH*HEIGHT];
		for (int i = 0; i < WIDTH*HEIGHT; i++) {
//			depthAvgs[i] = 0;
			validPixels[i] = -1;
			switchCount[i] = 0;
			depthAvgs[i] = 2047.0;
		}
		

		f = 580;
		cx = 5;
		cy = 17;

		kinect.setDepthFormat(DepthFormat.D11BIT);
		kinect.startDepth(new DepthHandler() {

		
			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer depthBuf, int _timestamp) {
				
				latestTimestamp = System.nanoTime();
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
							System.out.println("depth receiving error");
							depthBuf.position(0);
							return;
						}
						depth = byte2 & 0x7;
						depth = depth << 8;
						depth = depth | (byte1 & 0xFF);

						//background subtraction
						validPixels[i] = -1;
						boolean valid = false;
						if(depth < MAX_DEPTH)
						{
							if (depthAvgs[i] == 2047) {
								depthAvgs[i] = depth;
							}
							if ((depthAvgs[i] - (double)depth) > THRESH) {
								valid = true;
								validPixels[i] = (short)depth;		
								switchCount[i]++;	
							}
							//double depthFactor = (((depthAvgs[i] * (1.0-pg.gd("learning")) * (double)numFrames) + (pg.gd("learning")*(double)depth)) / (double)(numFrames + 1));
							depthAvgs[i] = (((depthAvgs[i] * (double)numFrames) + (double)depth) / (double)(numFrames + 1));
						}
						else {
							depthAvgs[i] = (((depthAvgs[i] * (double)numFrames) + (double)depthAvgs[i]) / (double)(numFrames + 1));
						}
						// if (botLoc != null) {
						// 	if (i == WIDTH*botLoc.y + botLoc.x) {
						// 		Point estimatedPicPoint = new Point(botLoc.x-C_X,C_Y-botLoc.y);
						// 		Point3D estimate = getWorldCoords(estimatedPicPoint,raw_depth_to_meters(depth));
						// 		System.out.println("x: "+ estimate.x +" y: "+ estimate.y +" z: " + estimate.z);
						// 		System.out.println("botLoc depth: " + depth + ", avg: " + depthAvgs[i]);
						// 	}
						// }




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

								if (showAll || valid) {
									pixelInts[i] = depthColor;
								}
								else {
									pixelInts[i] = 0xFF000000;
								}
							}
							catch(Exception e)
							{
								System.out.println("pixelInts null errror");
								depthBuf.position(0);
								return;
							}
						}
					
					}
					//set position to 0 because ByteBuffer is reused to access byte array of new frame
					//and get() below increments the iterator
					depthBuf.position(0);
					synchronized(imgMonitor) {
						imgMonitor.notify();
					}
					numFrames = (numFrames + 1) % MAX_FRAMES;
					if (display) {
						frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);

						for (Statistics ballpoints : trajectory) {
							Point depthPix = ballpoints.center();
							for (int y = depthPix.y - 3; y < depthPix.y + 3; y++) {
								for (int x = depthPix.x - 3; x < depthPix.x + 3; x++) {
									try {
										frame.setRGB(x, y, 0xFFFFFFFF);
									} catch (Exception e) {
										// System.out.println(x + " " + y);
									};
								}
							}
						}

						repaint();		
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
	//origin at top left
	public Point3D getWorldCoords(Point p) {
		Point pPix = new Point();
		pPix.x = p.x - C_X;
		pPix.y = C_Y - p.y;
		double depth = (double)getDepthFromDepthPixel(p);
		return getWorldCoords(pPix, depth);
	}
	public void showAll() {
		showAll = true;
	}
	public void showSubtraction() {
		showAll = false;
	}

	public short [] getValidImageArray() {
		return validPixels;
	}
	public void resetSwitches() {
		for (int i = 0; i < WIDTH*HEIGHT; i++) {
			// depthAvgs[i] = 0;
			// validPixels[i] = -1;
			switchCount[i] = 0;
		}
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
