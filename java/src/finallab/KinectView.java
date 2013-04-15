package finallab;

import java.io.*;
import java.nio.ByteBuffer;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import javax.imageio.*;

import org.openkinect.freenect.*;

import april.util.*;

import finallab.lcmtypes.*;

import java.util.concurrent.locks.*;

import lcm.lcm.*;

public class KinectView extends Thread
{
	Context ctx;
	Device kinect;

	JFrame jf;

	JImage rgbJim;
	JImage depthJim;

	JButton startTracking;
	boolean tracking = false;

	ParameterGUI pg;

	BufferedImage rgbImg;
	BufferedImage depthImg;

	Statistics BALL;
	ArrayList<Statistics> trajectory;
	boolean[] validImageValue;
	
	static double x_param = 0d;
	static int y_param = 0;

	LCM lcm;
	
	final boolean colorAnalyze = false;
	final boolean colorAnalyze2 = true;
	boolean display = false;

	volatile long globalTime = 0;
	volatile long startTime = System.nanoTime();
	
	final boolean verbose = false;

	volatile boolean newImage = false;

	int width = 640;
	int height = 480;

	BallTracker finder;

	KinectRGBVideo colorStream;
	KinectDepthVideo depthStream;

	int intoDepthX(int rgbX) {
    	// return (int)((double)Math.abs(x - 46)/586*640);
    	return rgbX + (int)(rgbX * x_param);
	}

	int intoDepthY(int rgbY) {
    	// return (int)((double)Math.abs(y - 37)/436*480);
    	return rgbY - (int)(((480 - rgbY) * y_param));
	}

	KinectView(boolean _display)
	{
		if(_display)
		{
			display = _display;
			System.out.println("Display Mode On");
			jf = new JFrame("KinectView");
			rgbJim = new JImage();
			depthJim = new JImage();
			startTracking = new JButton("Start Tracking Balls");
			startTracking.addActionListener(new ActionListener() {

				public void actionPerformed(ActionEvent e)
				{
					if(!tracking)
					{
						tracking = true;
						colorStream.pause();
						depthStream.pause();
						startTracking.setText("Stop Tracking");
					}
					else
					{
						tracking = false;
						colorStream.resume();
						depthStream.resume();
						startTracking.setText("Start Tracking Balls");
					}
				}
			});    
			if(colorAnalyze)
			{
				jf.setLayout(new GridLayout(2,2));
				pg = new ParameterGUI();
				pg.addIntSlider("redValMin","Red Min",0,255,0);
				pg.addIntSlider("redValMax","Red Max",0,255,255);
				pg.addIntSlider("greenValMin","Blue Min",0,255,0);
				pg.addIntSlider("greenValMax","Blue Max",0,255,255);
				pg.addIntSlider("blueValMin","Green Min",0,255,0);
				pg.addIntSlider("blueValMax","Green Max",0,255,255);		
				pg.addDoubleSlider("x_param","x_param",0.0,50,0);
				pg.addIntSlider("y_param","y_param",0,30,27);
				jf.add(pg, 1,0);
				jf.add(rgbJim, 0, 0);
				jf.add(depthJim, 0, 1);
				jf.setSize(1280,960);
			}
			else if(colorAnalyze2)
			{
				jf.setLayout(new GridLayout(2,2));
				pg = new ParameterGUI();
				//tennis ball
				// pg.addDoubleSlider("HueMin","Hue Min",-.2,1.0,0.135424);
				// pg.addDoubleSlider("HueMax","Hue Max",-.2,1.0,0.187456);
				// pg.addDoubleSlider("SatMin","Saturation Min",0,1.0,0.320480);
				// pg.addDoubleSlider("SatMax","Saturation Max",0,1.0,0.773490);
				// pg.addDoubleSlider("BrightMin","Brightness Min",0,1.0,0);
				// pg.addDoubleSlider("BrightMax","Brightness Max",0,1.0,1.0);
				//sky ball
				pg.addDoubleSlider("HueMin","Hue Min",-.2,1.0,0.2048);
				pg.addDoubleSlider("HueMax","Hue Max",-.2,1.0,0.4534);
				pg.addDoubleSlider("SatMin","Saturation Min",0,1.0,0.3228);
				pg.addDoubleSlider("SatMax","Saturation Max",0,1.0,1.0);
				pg.addDoubleSlider("BrightMin","Brightness Min",0,1.0,.313);
				pg.addDoubleSlider("BrightMax","Brightness Max",0,1.0,1.0);
				pg.addDoubleSlider("x_param","x_param",0d,1d,1d);
				pg.addIntSlider("y_param","y_param",1,500,100);
				pg.addIntSlider("blobThresh", "blob thresh", 1, 500, 125);
				jf.add(pg, 1,0);
				jf.add(rgbJim, 0, 0);
				jf.add(depthJim, 0, 1);
				jf.setSize(1280,960);
			}
			else
			{
				jf.setLayout(new GridLayout(1,2));
				jf.add(rgbJim, 0, 0);
				jf.add(depthJim, 0, 1);
				jf.setSize(1280,480);
			}


			jf.add(startTracking);
			jf.setVisible(true);
			jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		}

		rgbImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		depthImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);

		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
			return;
		}

		validImageValue = new boolean[width*height];


		lcm = LCM.getSingleton();
		BALL = new Statistics();
		trajectory = new ArrayList<Statistics>();

		finder = new BallTracker(width,height,true);
		
		double params[] = new double[6];
		params[0] = 0;
		params[1] = 1.0;
		params[2] = 0;
		params[3] = 1.0;
		params[4] = 0;
		params[5] = 1.0;
		kinect.setDepthFormat(DepthFormat.D11BIT);

		System.out.println("initializing depth stream");
		depthStream = new KinectDepthVideo(kinect,display);
		System.out.println("initializing rgb stream");
		colorStream = new KinectRGBVideo(kinect,0,params);
		System.out.println("done creating rgb stream");

		if(display)
		{
			depthImg = depthStream.getFrame();
			rgbImg = colorStream.getFrame();
		}
		
		// Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() 
		// {
  //   		// public void run() 
  //   		// {
  //   		// 	// close down kinect
  //   		// 	if (ctx != null)
  //   		// 		if (kinect != null) {
  //   		// 			kinect.close();
  //   		// 		}
  //   		// 		ctx.shutdown();
  //   		// 	System.out.println("Closed Down Kinect");

  //   		// }
		// }));
	}
	
	public void run() {
		int ballNum = 0;
		while (true) {
			BALL = null;
			ball_t ballLCM = new ball_t();
			double params[] = new double[6];
			if (display) {
				rgbJim.setImage(rgbImg);
				depthJim.setImage(depthImg);
				if (colorAnalyze) {
					params[0] = (double) pg.gi("redValMin");
					params[2] = (double) pg.gi("greenValMin");
					params[4] = (double) pg.gi("blueValMin");
					params[1] = (double) pg.gi("redValMax");
					params[3] = (double) pg.gi("greenValMax");
					params[5] = (double) pg.gi("blueValMax");
					x_param = pg.gd("x_param");
					y_param = pg.gi("y_param");
					colorStream.changeThreshold((byte) 2, params);
				} else if (colorAnalyze2) {
					params[0] = pg.gd("HueMin");
					params[1] = pg.gd("HueMax");
					params[2] = pg.gd("SatMin");
					params[3] = pg.gd("SatMax");
					params[4] = pg.gd("BrightMin");
					params[5] = pg.gd("BrightMax");
					x_param = pg.gd("x_param");
					y_param = pg.gi("y_param");
					colorStream.changeThreshold((byte) 1, params);
				}
			}

			while (!depthStream.newImage);
			ballLCM.nanoTime = System.nanoTime();
			depthStream.newImage = false;
			ArrayList<Statistics> blobs;
			depthStream.getReadLock().lock();
			try {
				blobs = finder.analyze2(depthStream
						.getValidImageArray());
			}
			finally {
				depthStream.getReadLock().unlock();
			}
			Statistics ball = null;
			Statistics robot = null;
			int minSize = pg.gi("blobThresh");

			//find robot and ball by blob size
			Collections.sort(blobs, ComparatorFactory.getStatisticsCompareSize());
			//find robot and ball by y pixel
			// Collections.sort(blobs, ComparatorFactory.getStatisticsCompareYPix());

			if (blobs.size() == 1) {
				Statistics first = blobs.get(0);
				if (first.N > minSize) {
					ball = first;
				}
			}
			else if (blobs.size() >= 2) {
				Statistics first = blobs.get(0);
				Statistics second = blobs.get(1);
				if (first.N > minSize) {
					ball = first;
				}
				if (second.N > minSize) {
					robot = first;
					ball = second;
				}
			}


			if (ball != null)
				trajectory.add(ball);
			for (Statistics ballpoints : trajectory) {
				Point depthPix = ballpoints.center();
				for (int y = depthPix.y - 3; y < depthPix.y + 3; y++) {
					for (int x = depthPix.x - 3; x < depthPix.x + 3; x++) {
						try {
							depthImg.setRGB(x, y, 0xFFFFFFFF);
						} catch (Exception e) {
							// System.out.println(x + " " + y);
						};
					}
				}
			}
			// System.out.println("balls points " + trajectory.size());


			// if not tracking keep kv.trajectory to just one index
			if (!tracking) {
				trajectory.clear();
			}

			if (ball != null) {

				Point depthPix = ball.center();
				Point depthCoord = new Point();
				depthCoord.x = depthPix.x - KinectVideo.C_X;
				depthCoord.y = KinectVideo.C_Y - depthPix.y;
				// System.out.println("Depth at center: " + depthStream.getValidImageArray()[depthPix.y*640+depthPix.x]);
				// Point depthCoord = new Point(depthPix.x - KinectVideo.C_X,
				// KinectVideo.C_Y - depthPix.y);
				// System.out.println("center depth " + depthStream.getDepthValFromDepthPixel(depthPix));
				// System.out.println("avg depth " + ball.Uz());
				double realDepth = raw_depth_to_meters(ball.Uz());
				Point3D coord = depthStream.getWorldCoords(depthCoord, realDepth);
				if (depthPix != null) {
					// System.out.println("depth blobs: " + depthBlobs.size());
					// System.out.println("time diff: " +
					// (depthStream.getLatestTime() -
					// colorStream.getLatestTime()));
					// System.out.println("depthPix: " + depthPix.x + ", " +
					// depthPix.y);
					for (int y = depthPix.y - 3; y < depthPix.y + 3; y++) {
						for (int x = depthPix.x - 3; x < depthPix.x + 3; x++) {
							try {
								depthImg.setRGB(x, y, 0xFFFF0000);
							} catch (Exception e) {
								// System.out.println(x + " " + y);
							};
						}
					}
					// if (tracking) {
					// 	//save image
					// 	depthStream.getReadLock().lock();
					// 	try {
					// 		File imgFile = new File("image" + ballNum++ + ".png");
					// 		try {
					// 			ImageIO.write(depthImg, "png", imgFile);
					// 		}
					// 		catch(Exception e) {
					// 			System.out.println("can't save img");
					// 		}
					// 	}
					// 	finally {
					// 		depthStream.getReadLock().unlock();
					// 	}
					// }

					try {
						// coord.z =
						// depthStream.getDepthFromDepthPixel(depthPix);
						// System.out.println("depth: " + coord.z);
					} catch (Exception e) {

					}
					// System.out.println("depth: " +
					// depthStream.getDepthFromDepthPixel(depthPix));
					if (tracking) {
						ballLCM.x = coord.x;
						ballLCM.y = coord.y;
						ballLCM.z = coord.z;
						// if(tracking)
						lcm.publish("6_BALL", ballLCM);
					}
				}
				// try
			}

			// {
			// // Point3D realWorld =
			// depthStream.getWorldCoords(ClosestBall.closestPixel);
			// // System.out.println(realWorld.x + " " + realWorld.y + " " +
			// realWorld.z);
			// }
			// catch(Exception e){};
			depthJim.setImage(depthImg);
			if (display) {
				rgbJim.setImage(rgbImg);
				
				if (colorAnalyze) {
					params[0] = (double) pg.gi("redValMin");
					params[2] = (double) pg.gi("greenValMin");
					params[4] = (double) pg.gi("blueValMin");
					params[1] = (double) pg.gi("redValMax");
					params[3] = (double) pg.gi("greenValMax");
					params[5] = (double) pg.gi("blueValMax");
					x_param = pg.gd("x_param");
					y_param = pg.gi("y_param");
					colorStream.changeThreshold((byte) 2, params);
				} else if (colorAnalyze2) {
					params[0] = pg.gd("HueMin");
					params[1] = pg.gd("HueMax");
					params[2] = pg.gd("SatMin");
					params[3] = pg.gd("SatMax");
					params[4] = pg.gd("BrightMin");
					params[5] = pg.gd("BrightMax");
					x_param = pg.gd("x_param");
					y_param = pg.gi("y_param");
					colorStream.changeThreshold((byte) 1, params);
				}
			}
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

	//utility functions to copy/paste later	
	public int getDepth(ByteBuffer bb, int index) 
	{
		int depth = 0;
		byte byte1 = bb.get(index * 2);
		byte byte2 = bb.get(index * 2 + 1);
		depth = byte2 & 0x3;
		depth = depth << 8;
		depth = depth | (byte1 & 0xFF);
		return depth & 0x3FF;
	}

}
