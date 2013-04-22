package finallab;

import java.io.IOException;
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

public class BallDetector extends Thread
{
	Context ctx;
	Device kinect;

	JFrame controlFrame;
	JFrame colorFrame;
	JFrame depthFrame;

	// JImage rgbJim;
	// JImage depthJim;

	JButton startTracking;
	JButton resetProjectile;
	JButton resetDepth;
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

	KinectRGBVideo colorStream;
	Object colorMonitor;
	KinectDepthVideo depthStream;
	Object depthMonitor;

	BallDetector(boolean _display)
	{
		
		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
			return;
		}
		display = _display;
		
		controlFrame = new JFrame("Controls");
		controlFrame.setLayout(new GridLayout(5,1));
		controlFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		pg = new ParameterGUI();
		pg.addIntSlider("maxDepth", "max depth", 800, 2047, 1050);
		pg.addIntSlider("blobThresh", "blob thresh", 1, 500, 125);
		pg.addIntSlider("thresh", "thresh", 1, 100, 10);
		pg.addIntSlider("frames", "frames", 1, 1000, 1);
		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI _pg, String name) {
				if (name.equals("thresh")) {
					KinectDepthVideo.THRESH = _pg.gi(name);
				}
				else if (name.equals("frames")) {
					KinectDepthVideo.MAX_FRAMES = _pg.gi(name);
				}
				else if (name.equals("maxDepth")) {
					KinectDepthVideo.MAX_DEPTH = _pg.gi(name);
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

		resetProjectile = new JButton ("Reset Projectile");
		resetProjectile.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				try {
					lcm.publish("6_RESET", "reset");
				}
				catch(IOException ex) {
					System.out.println("can't publish reset");
				}
			}
		});  
		controlFrame.add(resetProjectile, 2, 0);
		resetDepth = new JButton ("Reset Depth Avgs");
		resetDepth.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				DepthClearer ic = new DepthClearer(pg);
				ic.start();
			}
		});
		controlFrame.add(resetDepth, 3, 0);
		JPanel scoreButtons = new JPanel(new GridLayout(1,3));
		JButton addHuman = new JButton("human++");
		addHuman.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				try {
					lcm.publish("6_SCORE_HUMAN", "bish");
				}
				catch(IOException ex) {
					System.out.println("can't publish score");
				}
			}
		});
		JButton addRobot = new JButton("robot++");
		addRobot.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				try {
					lcm.publish("6_SCORE_ROBOT", "bish");
				}
				catch(IOException ex) {
					System.out.println("can't publish score");
				}
			}
		});
		JButton resetScores = new JButton("reset scores");
		resetScores.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				try {
					lcm.publish("6_SCORE_RESET", "bish");
				}
				catch(IOException ex) {
					System.out.println("can't publish score");
				}
			}
		});
		scoreButtons.add(addHuman, 0, 0);
		scoreButtons.add(addRobot, 0, 1);
		scoreButtons.add(resetScores, 0, 2);
		controlFrame.add(scoreButtons, 4, 0);
		controlFrame.setSize(800, 600);
		controlFrame.setVisible(true);

		colorFrame = new JFrame("color feed");
		colorMonitor = new Object();
		colorStream = new KinectRGBVideo(kinect, colorMonitor, display);
		colorFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		colorFrame.addWindowListener(new RGBClose());
		colorFrame.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		colorFrame.setContentPane(colorStream);
		colorFrame.setVisible(true);

		depthFrame = new JFrame("depth feed");
		depthMonitor = new Object();
		depthStream = new KinectDepthVideo(kinect, depthMonitor, display);
		depthFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		depthFrame.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		depthFrame.setContentPane(depthStream);
		depthFrame.setVisible(true);




		rgbImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		depthImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);

		validImageValue = new boolean[KinectVideo.WIDTH*KinectVideo.HEIGHT];
		try{
			lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}
		catch(IOException e){
			lcm = LCM.getSingleton();
		}
		BALL = new Statistics();

		finder = new BallTracker(KinectVideo.WIDTH,KinectVideo.HEIGHT,false);

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
				depthStream.botLoc = botPix;
			}
		});
		DepthClearer ic = new DepthClearer(pg);
		ic.start();

	}
	
	//stops rgb feed if user closes rgb window
	public class RGBClose implements WindowListener {

		public void windowClosing(WindowEvent e) {
			kinect.stopVideo();
			colorFrame.dispose();
		}
		public void windowOpened(WindowEvent e){   }
      	public void windowClosed(WindowEvent e){   }
	    public void windowActivated(WindowEvent e){   }
	    public void windowDeactivated(WindowEvent e){   }
	    public void windowIconified(WindowEvent e){   }
	    public void windowDeiconified(WindowEvent e){   }
	}
	
	public void run() {
		while (true) {
			BALL = null;
			ball_t ballLCM = new ball_t();

			synchronized(depthMonitor) {
				try {
					depthMonitor.wait();
				}
				catch(Exception e) {
					e.printStackTrace();
				}
			}
			ballLCM.nanoTime = depthStream.latestTimestamp;
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
//			if (tracking) {
//				System.out.println("num blobs: " + blobs.size());
//				System.out.println("biggest blob size: " + blobs.get(0).N);
//			}
//			for (Statistics blob : blobs) {
//				if (blob.N > 10) {
//					System.out.println("blob size: " + blob.N);
//				}
//				else {
//					break;
//				}
//			}
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
				
				// System.out.println("avg depth " + ball.Uz());
				
				//get depth from average depth of blob
				double realDepth = raw_depth_to_meters(ball.Uz());
				Point3D coord = depthStream.getWorldCoords(depthCoord, realDepth);
				if (depthPix != null) {
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

					if (tracking) {
						ballLCM.x = coord.x;
						ballLCM.y = coord.y;
						ballLCM.z = coord.z;
						// if(tracking)
						System.out.println("updating new ball (" + System.currentTimeMillis() + ")");
//						if (ballLCM.x > CatchController.TARGET_MAX_X)
							lcm.publish("6_BALL", ballLCM);
//						else 
//							System.out.println("ball past target zone");
					}
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
	public void setLog(boolean b) {
		log = b;
	}
	
	
	public class DepthClearer extends Thread {
		ParameterGUI pg;
		public DepthClearer(ParameterGUI _pg) {
			pg = _pg;
		}
		//set max_frames to 1 for 5 seconds then move up to 200
		public void run() {
			pg.si("frames", 1);
			try {
				Thread.sleep(5000);
			} catch (Exception e) {
				e.printStackTrace();
			}
			for (int i = 1; i <= 100; i++) {
				pg.si("frames", 2 * i);
				try {
					Thread.sleep(10);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}

	public void plotProjection(ArrayList<double[]> pballs)
	{
		for(int i = 0; i < pballs.size(); i++)
		{
			double xyzt[] = pballs.get(i);
			//switch y and z;
			double temp = xyzt[2];
			xyzt[2] = xyzt[1];
			xyzt[1] = temp;
			Point3D realWorld = new Point3D(xyzt[0],xyzt[1],xyzt[2]);
			//Point pixCoords = depthgetPixFromWorld(realWorld);
			//depthImg.setRGB(pixCoords.x,pixCoords.y,0xFFFFFFFF);
		}
	}

	public static void main(String [] args) {
//		Projectile proj = new Projectile();
		BallDetector kv = new BallDetector(true);
//		for(int i = 0; i < args.length; i++)
//		{
//			if(args[i].equals("log"))
//				kv.setLog(true);
//		}
		kv.start();
	}

}
