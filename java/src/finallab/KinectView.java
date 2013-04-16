package finallab;

import java.nio.ByteBuffer;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import org.openkinect.freenect.*;

import april.util.*;

import finallab.lcmtypes.*;

import lcm.lcm.*;

public class KinectView extends Thread
{
	Context ctx;
	Device kinect;

	JFrame controlFrame;
	JFrame colorFrame;
	JFrame depthFrame;

	// JImage rgbJim;
	// JImage depthJim;

	JButton startTracking;
	JButton resetSwitches;
	boolean tracking = false;
	boolean log = false;

	ParameterGUI pg;

	BufferedImage rgbImg;
	BufferedImage depthImg;

	Statistics BALL;
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
	public Point3D botStart;

	BallTracker finder;
	Projectile predictor;

	KinectRGBVideo colorStream;
	KinectDepthVideo depthStream;

	KinectView(Projectile _predictor, boolean _display)
	{

		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
			return;
		}
		predictor = _predictor;
		display = _display;

		controlFrame = new JFrame("Controls");
		controlFrame.setLayout(new GridLayout(3,1));
		controlFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		pg = new ParameterGUI();
		pg.addDoubleSlider("x_param","x_param",0d,1d,1d);
		pg.addIntSlider("y_param","y_param",1,500,100);
		pg.addIntSlider("blobThresh", "blob thresh", 1, 500, 125);
		pg.addIntSlider("thresh", "thresh", 1, 100, 10);
		pg.addIntSlider("frames", "frames", 1, 1000, 1);
		pg.addIntSlider("switches", "max switches", 1, 1000, 1000);
		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI _pg, String name) {
				if (name.equals("thresh")) {
					depthStream.THRESH = _pg.gi(name);
				}
				else if (name.equals("frames")) {
					depthStream.MAX_FRAMES = _pg.gi(name);
				}
				else if (name.equals("switches")) {
					depthStream.SWITCHES = _pg.gi(name);
				}
			}
		});
		controlFrame.add(pg, 0,0);

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
		controlFrame.add(startTracking, 1, 0);

		resetSwitches = new JButton ("Reset Switches");
		resetSwitches.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				depthStream.resetSwitches();
			}
		});  
		controlFrame.add(resetSwitches, 3, 0);
		controlFrame.setSize(800, 200);
		controlFrame.setVisible(true);

		colorFrame = new JFrame("color feed");
		colorStream = new KinectRGBVideo(kinect,display);
		colorFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		colorFrame.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		colorFrame.setContentPane(colorStream);
		colorFrame.setVisible(true);

		depthFrame = new JFrame("depth feed");
		depthStream = new KinectDepthVideo(kinect,display);
		depthFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		depthFrame.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		depthFrame.setContentPane(depthStream);
		depthFrame.setVisible(true);




		rgbImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		depthImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);

		validImageValue = new boolean[KinectVideo.WIDTH*KinectVideo.HEIGHT];
		if (log)
			lcm = LCM.getSingleton();
		BALL = new Statistics();

		finder = new BallTracker(KinectVideo.WIDTH,KinectVideo.HEIGHT,true);

		if(display)
		{
			depthImg = depthStream.getFrame();
			rgbImg = colorStream.getFrame();
		}
		//get robot position from click
		depthStream.addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				Point botPix = e.getPoint();
				botStart = depthStream.getWorldCoords(botPix);	
				botStart.z += 0.08;
				System.out.println("botStart: " + botStart.toString());
				depthStream.showSubtraction();		
			}
		});

	}
	
	public void run() {
		while (true) {
			BALL = null;
			ball_t ballLCM = new ball_t();

			while (!depthStream.newImage);
			ballLCM.nanoTime = System.nanoTime();
			depthStream.newImage = false;
			ArrayList<Statistics> blobs;
			depthStream.getReadLock().lock();
			try {
				blobs = finder.analyze2(depthStream.getValidImageArray());
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
			
			// System.out.println("balls points " + depthStream.trajectory.size());


			// if not tracking keep kv.depthStream.trajectory to just one index
			if (!tracking) {
				depthStream.trajectory.clear();
			}

			if (ball != null) {
				depthStream.trajectory.add(ball);

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

					// System.out.println("depth: " +
					// depthStream.getDepthFromDepthPixel(depthPix));
					if (tracking) {
						ballLCM.x = coord.x;
						ballLCM.y = coord.y;
						ballLCM.z = coord.z;
						// if(tracking)
						predictor.update(ballLCM);
						if (log) {
							lcm.publish("6_BALL", ballLCM);
						}
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
	public void setLog(boolean b) {
		log = b;
	}

	public static void main(String [] args) {
		Projectile proj = new Projectile();
		KinectView kv = new KinectView(proj, true);
		for(int i = 0; i < args.length; i++)
		{
			if(args[i].equals("log"))
				kv.setLog(true);
		}
		kv.start();
	}

}
