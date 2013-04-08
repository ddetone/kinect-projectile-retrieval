package finallab;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import org.openkinect.freenect.*;
import org.openkinect.freenect.util.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;
import april.jmat.geom.*;

import finallab.lcmtypes.*;
import lcm.lcm.*;

public class KinectView
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
						startTracking.setText("Stop Tracking");
					}
					else
					{
						tracking = false;
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
		}

		validImageValue = new boolean[width*height];

		lcm = LCM.getSingleton();

		BALL = new Statistics();
		trajectory = new ArrayList<Statistics>();

		finder = new BallTracker(width,height,display);
		
		double params[] = new double[6];
		params[0] = 0;
		params[1] = 1.0;
		params[2] = 0;
		params[3] = 1.0;
		params[4] = 0;
		params[5] = 1.0;
		kinect.setDepthFormat(DepthFormat.D11BIT);

		colorStream = new KinectRGBVideo(kinect,1,params);
		depthStream = new KinectDepthVideo(kinect,display);
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

	public ball_t update()
	{
		BALL = null;
		double params[] = new double[6];
		if(display)
		{
			rgbJim.setImage(rgbImg);
			depthJim.setImage(depthImg);
			if(colorAnalyze && display)
			{
				params[0] = (double)pg.gi("redValMin");
				params[2] = (double)pg.gi("greenValMin");
				params[4] = (double)pg.gi("blueValMin");
				params[1] = (double)pg.gi("redValMax");
				params[3] = (double)pg.gi("greenValMax");
				params[5] = (double)pg.gi("blueValMax");
				x_param = pg.gd("x_param");
				y_param = pg.gi("y_param");
				colorStream.changeThreshold((byte)2,params);
			}
			else if(colorAnalyze2 && display)
			{
				params[0] = pg.gd("HueMin");
				params[1] = pg.gd("HueMax");
				params[2] = pg.gd("SatMin");
				params[3] = pg.gd("SatMax");
				params[4] = pg.gd("BrightMin");
				params[5] = pg.gd("BrightMax");
				x_param = pg.gd("x_param");
				y_param = pg.gi("y_param");
				colorStream.changeThreshold((byte)1,params);
			}
		}

		while(!colorStream.newImage && !depthStream.newImage);
		ball_t ballLCM = new ball_t();
		ballLCM.nanoTime = System.nanoTime();
		colorStream.newImage = false;
		depthStream.newImage = false;
		// Point poi = new Point();
		// poi.x = 320;
		// poi.y = 240;
		// ArrayList<Statistics> blobs = finder.analyzePartition(colorStream.getValidImage(), poi, 640,480, "center");
		// Statistics BiggestBlob = new Statistics();
		// for(Statistics blob : blobs)
		// {
		// 	if(blob.N > 50)
		// 	{
		// 		if(BiggestBlob.N < blob.N)
		// 			BiggestBlob = blob;
		// 		blob.center();
		// 	}
		// 	BALL = BiggestBlob;
		// 	BALL.center();

		// }
		// trajectory.add(BiggestBlob);
		// for(Statistics ball : trajectory)
		// {
		// 	for(int y = ball.center_y-3; y < ball.center_y+3; y++)
		// 	{
		// 		for(int x = ball.center_x-3; x < ball.center_x+3;x++)
		// 		{
		// 			try{
		// 				rgbImg.setRGB(x,y,0xFF000000);
		// 			}
		// 			catch(Exception e){};	
		// 		}
		// 	}
		// }

		ArrayList<Statistics> blobs = finder.analyze2(depthStream.getValidImageArray());
		Statistics ball = null;

		for (Statistics curr : blobs) {
			if (ball == null) {
				ball = curr;
				continue;
			}
			if (curr.N > ball.N) {
				System.out.println("blob size: " + curr.N + ", blob abs: " + curr.abs());
				ball = curr;
			}
			
		}

		//if not tracking keep kv.trajectory to just one index
		if(!tracking)
		{
			trajectory.clear();
		}

		//draw bounding box to determine if ball will fall in place
		// int bound = (int)(x_param*160.0);
		// try
		// {
		// 	for(int y = BALL.center_y-(bound/2); y < BALL.center_y+(bound/2); y++)
		// 	{
		// 		// depthImg.setRGB(BALL.center_x-(bound/2),y,0xFFFFFFFF);
		// 		// depthImg.setRGB(BALL.center_x+(bound/2),y,0xFFFFFFFF);
		// 	}
		// 	for(int x = BALL.center_x-(bound/2); x < BALL.center_x+(bound/2); x++)
		// 	{
		// 		// depthImg.setRGB(x,BALL.center_y-(bound/2),0xFFFFFFFF);
		// 		// depthImg.setRGB(x,BALL.center_y+(bound/2),0xFFFFFFFF);
		// 	}
		// }
		// catch(Exception e){};
		
		if(ball != null)
		{
			// ClosestBall.center();
			// for(int y = ClosestBall.closestPixel.y-3; y < ClosestBall.closestPixel.y+3; y++)
			// {
			// 	for(int x = ClosestBall.closestPixel.x-3; x < ClosestBall.closestPixel.x+3;x++)
			// 	{
			// 		try{
			// 			// depthImg.setRGB(x,y,0xFFFFFFFF);
			// 		}
			// 		catch(Exception e){};
			// 	}
			// }
			// System.out.println("x-pix diff: " + (ClosestBall.max_x - ClosestBall.min_x));

			// Point3D coord = depthStream.getWorldCoords(ClosestBall.closestPixel);
			// Point ballCoord = new Point(ball.center_x - KinectVideo.C_X, KinectVideo.C_Y - ball.center_y);
			// System.out.println("ballCoord: " + ballCoord.x + ", " + ballCoord.y);
			// double depthGuess = colorStream.guessDepth(ball.max_x - ball.min_x);
			// System.out.println("guess: " + depthGuess);



			// Point3D coord = colorStream.getWorldCoords(ballCoord, colorStream.guessDepth(ball.max_x - ball.min_x));
			// coord.x += KinectVideo.RGB_DEPTH_DIST;
			// Point depthCoord = depthStream.getPixFromWorld(coord);
			// // System.out.println("depthCoord: " + depthCoord.x + ", " + depthCoord.y);
			// Point depthPix = new Point(depthCoord.x + KinectVideo.C_X, KinectVideo.C_Y - depthCoord.y);
			// // depthPix.y -= y_param;
			// depthPix.y -= 16;
			// depthPix.x -= depthPix.x * .003;

			// int ballWidth = ball.max_x - ball.min_x;

			// int ballBound = ballWidth * 2;

			// int ballArea = (int)(3.14 * (ballWidth/2) * (ballWidth/2));

			// ArrayList<Statistics> depthBlobs = finder.analyzeDepthPartition(depthStream.getBuf(),depthPix, ballBound, ballArea);
			// // System.out.println("num blobs " + depthBlobs.size());
			// Statistics ClosestBall = new Statistics();
			// for(Statistics blob : depthBlobs)
			// {
			// 	if(blob.closestDepth < ClosestBall.closestDepth)
			// 	{
			// 		ClosestBall = blob;
			// 	}
			// }
			// depthPix = ClosestBall.closestPixel;
			Point depthPix = ball.center();
			Point depthCoord = new Point(depthPix.x - KinectVideo.C_X, KinectVideo.C_Y - depthPix.y);
			Point3D coord = depthStream.getWorldCoords(depthPix);
			if (depthPix != null) {
				// System.out.println("depth blobs: " + depthBlobs.size());
				// System.out.println("time diff: " + (depthStream.getLatestTime() - colorStream.getLatestTime()));
				//System.out.println("depthPix: " + depthPix.x + ", " + depthPix.y);
				for(int y = depthPix.y-3; y < depthPix.y+3; y++)
				{
					for(int x = depthPix.x-3; x < depthPix.x+3;x++)
					{
						try{
							depthImg.setRGB(x,y,0xFFFF0000);
						}
						catch(Exception e){
							// System.out.println(x + " " + y);
						};
					}
				}
				try {
					// coord.z = depthStream.getDepthFromDepthPixel(depthPix);
					// System.out.println("depth: " + coord.z);
				}
				catch(Exception e) {

				} 
				// System.out.println("depth: " + depthStream.getDepthFromDepthPixel(depthPix));
				ballLCM.x = coord.x;
				ballLCM.y = coord.y + 0.84;
				ballLCM.z = coord.z;
				//if(tracking)
					lcm.publish("6_BALL",ballLCM);
			}
		// try
		}


		// {
		// 	// Point3D realWorld = depthStream.getWorldCoords(ClosestBall.closestPixel);
		// 	// System.out.println(realWorld.x + " " + realWorld.y + " " + realWorld.z);
		// }
		// catch(Exception e){};
		if(display)
		{
			rgbJim.setImage(rgbImg);
			depthJim.setImage(depthImg);
			if(colorAnalyze && display)
			{
				params[0] = (double)pg.gi("redValMin");
				params[2] = (double)pg.gi("greenValMin");
				params[4] = (double)pg.gi("blueValMin");
				params[1] = (double)pg.gi("redValMax");
				params[3] = (double)pg.gi("greenValMax");
				params[5] = (double)pg.gi("blueValMax");
				x_param = pg.gd("x_param");
				y_param = pg.gi("y_param");
				colorStream.changeThreshold((byte)2,params);
			}
			else if(colorAnalyze2 && display)
			{
				params[0] = pg.gd("HueMin");
				params[1] = pg.gd("HueMax");
				params[2] = pg.gd("SatMin");
				params[3] = pg.gd("SatMax");
				params[4] = pg.gd("BrightMin");
				params[5] = pg.gd("BrightMax");
				x_param = pg.gd("x_param");
				y_param = pg.gi("y_param");
				colorStream.changeThreshold((byte)1,params);
			}
		}
		return ballLCM;
	}

	public static void main(String[] args)
	{
		final KinectView kv = new KinectView(true);
		double params[] = new double[6];
		if(kv.colorAnalyze && kv.display)
		{
			params[0] = (double)kv.pg.gi("redValMin");
			params[2] = (double)kv.pg.gi("greenValMin");
			params[4] = (double)kv.pg.gi("blueValMin");
			params[1] = (double)kv.pg.gi("redValMax");
			params[3] = (double)kv.pg.gi("greenValMax");
			params[5] = (double)kv.pg.gi("blueValMax");
			x_param = kv.pg.gd("x_param");
			y_param = kv.pg.gi("y_param");
		}
		else if(kv.colorAnalyze2 && kv.display)
		{
			params[0] = kv.pg.gd("HueMin");
			params[1] = kv.pg.gd("HueMax");
			params[2] = kv.pg.gd("SatMin");
			params[3] = kv.pg.gd("SatMax");
			params[4] = kv.pg.gd("BrightMin");
			params[5] = kv.pg.gd("BrightMax");
			x_param = kv.pg.gd("x_param");
			y_param = kv.pg.gi("y_param");
		}
		while(true) 
		{
			while(!kv.colorStream.newImage && !kv.depthStream.newImage);
			// System.out.println("Got new Image");
			// System.out.println(System.currentTimeMillis());
			ball_t ballLCM = new ball_t();
			ballLCM.nanoTime = kv.globalTime;
			kv.colorStream.newImage = false;
			kv.depthStream.newImage = false;
			Point poi = new Point();
			poi.x = 320;
			poi.y = 240;
			ArrayList<Statistics> blobs = kv.finder.analyzePartition(kv.colorStream.getValidImage(), poi, 640,480, "center");
			Statistics BiggestBlob = new Statistics();
			for(Statistics blob : blobs)
			{
				if(blob.N > 50)
				{
					if(BiggestBlob.N < blob.N)
						BiggestBlob = blob;
					blob.center();
				}
				kv.BALL = BiggestBlob;
				kv.BALL.center();
				
			}
			kv.trajectory.add(BiggestBlob);
			for(Statistics ball : kv.trajectory)
			{
				for(int y = ball.center_y-3; y < ball.center_y+3; y++)
					for(int x = ball.center_x-3; x < ball.center_x+3;x++)
						try{
								kv.rgbImg.setRGB(x,y,0xFF000000);
						}
						catch(Exception e){};	
			}

			//if not tracking keep kv.trajectory to just one index
			if(!kv.tracking)
			{
				kv.trajectory.clear();
			}

			//draw bounding box to determine if ball will fall in place
			int bound = (int)(x_param*160.0);
			try
			{
				for(int y = kv.BALL.center_y-(bound/2); y < kv.BALL.center_y+(bound/2); y++)
				{
					kv.depthImg.setRGB(kv.BALL.center_x-(bound/2),y,0xFFFFFFFF);
					kv.depthImg.setRGB(kv.BALL.center_x+(bound/2),y,0xFFFFFFFF);
				}
				for(int x = kv.BALL.center_x-(bound/2); x < kv.BALL.center_x+(bound/2); x++)
				{
					kv.depthImg.setRGB(x,kv.BALL.center_y-(bound/2),0xFFFFFFFF);
					kv.depthImg.setRGB(x,kv.BALL.center_y+(bound/2),0xFFFFFFFF);
				}
			}
			catch(Exception e){};
				//System.println(kv.getDepth(kv.depthImg,kv.BALL.center_y*width+kv.BALL.center_x));

			ballLCM.x = kv.BALL.center_x;
			ballLCM.y = kv.BALL.center_y;
			ballLCM.z = 4;
			kv.lcm.publish("6_BALL",ballLCM);
			Point ballCenter = new Point(kv.BALL.center_x,kv.BALL.center_y);
			ArrayList<Statistics> depthBlobs = new ArrayList<Statistics>();//kv.finder.analyzeDepthPartition(kv.depthStream.getBuf(),ballCenter,bound);
			Statistics ClosestBall = new Statistics();
			for(Statistics blob : depthBlobs)
			{
				if(blob.closestDepth < ClosestBall.closestDepth)
				{
					ClosestBall = blob;
				}
			}
			if(ClosestBall.closestPixel != null)
			{
				ClosestBall.center();
				for(int y = ClosestBall.closestPixel.y-3; y < ClosestBall.closestPixel.y+3; y++)
						for(int x = ClosestBall.closestPixel.x-3; x < ClosestBall.closestPixel.x+3;x++)
							try{
								kv.depthImg.setRGB(x,y,0xFFFFFFFF);
							}
							catch(Exception e){};
				//System.out.println(ClosestBall.closestDepth);
			}
			try
			{
				Point3D realWorld = kv.depthStream.getWorldCoords(ClosestBall.closestPixel);
				System.out.println(realWorld.x + " " + realWorld.y + " " + realWorld.z);
			}
			catch(Exception e){};
			
			kv.rgbJim.setImage(kv.rgbImg);
			kv.depthJim.setImage(kv.depthImg);
			if(kv.colorAnalyze && kv.display)
			{
				params[0] = (double)kv.pg.gi("redValMin");
				params[2] = (double)kv.pg.gi("greenValMin");
				params[4] = (double)kv.pg.gi("blueValMin");
				params[1] = (double)kv.pg.gi("redValMax");
				params[3] = (double)kv.pg.gi("greenValMax");
				params[5] = (double)kv.pg.gi("blueValMax");
				x_param = kv.pg.gd("x_param");
				y_param = kv.pg.gi("y_param");
				kv.colorStream.changeThreshold((byte)2,params);
			}
			else if(kv.colorAnalyze2 && kv.display)
			{
				params[0] = kv.pg.gd("HueMin");
				params[1] = kv.pg.gd("HueMax");
				params[2] = kv.pg.gd("SatMin");
				params[3] = kv.pg.gd("SatMax");
				params[4] = kv.pg.gd("BrightMin");
				params[5] = kv.pg.gd("BrightMax");
				x_param = kv.pg.gd("x_param");
				y_param = kv.pg.gi("y_param");
				kv.colorStream.changeThreshold((byte)1,params);
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

	/*
	public void convertToHSV(){
		for(int j = height - searchHeight; j < height; j++){
			int tempj = j - height + searchHeight;
			for(int i = 0; i < width; i++){
				//System.out.println((height - j) * width + i);
				int aRGB = data[j * width + i];
				Color.RGBtoHSB(((aRGB & 0x00FF0000) >> 16), ((aRGB & 0x0000FF00) >> 8), (aRGB & 0x000000FF), hsvImage[tempj * width + i]);
				float temp = hsvImage[tempj * width + i][0];
				hsvImage[tempj * width + i][0] = (temp - (int)temp) * 360;
			}
		}
	}


	public double colorScore(int i, int j, int color){

		//System.out.println((height - (j + 1)) * width + i);
		float[] pixel = hsvImage[(j - height + searchHeight) * width + i];
		int distFromColor = Math.max(color, (int)pixel[0]) - Math.min(color, (int)pixel[0]);
		return(double)(360 - distFromColor);

	}

			//draw box at center of image
		// for(int y = (height/2)-5; y < (height/2)+5; y++)
		// 	for(int x = (width/2)-5; x < (width/2)+5;x++)
		// 		pixelInts[y*width+x] = 0xFFFFFFFF;
	*/
