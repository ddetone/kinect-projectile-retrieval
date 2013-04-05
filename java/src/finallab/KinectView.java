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
	static double y_param = 0d;

	LCM lcm;
	
	final boolean colorAnalyze = false;
	final boolean colorAnalyze2 = true;

	volatile long globalTime = 0;
	
	final boolean verbose = false;

	volatile boolean newImage = false;

	int width = 640;
	int height = 480;

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

			rgbImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
			depthImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);

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

				jf.add(pg, 1,0);
				jf.add(rgbJim, 0, 0);
				jf.add(depthJim, 0, 1);
				jf.setSize(1280,960);
			}
			else if(colorAnalyze2)
			{
				jf.setLayout(new GridLayout(2,2));
				pg = new ParameterGUI();
				pg.addDoubleSlider("HueMin","Hue Min",0,1.0,0);
				pg.addDoubleSlider("HueMax","Hue Max",0,1.0,1.0);
				pg.addDoubleSlider("SatMin","Saturation Min",0,1.0,0);
				pg.addDoubleSlider("SatMax","Saturation Max",0,1.0,1.0);
				pg.addDoubleSlider("BrightMin","Brightness Min",0,1.0,0);
				pg.addDoubleSlider("BrightMax","Brightness Max",0,1.0,1.0);

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
		
		rgbImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		depthImg = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);

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
			pg.addDoubleSlider("xParam", "xParam", 0, 0.1, 0);	
			pg.addDoubleSlider("yParam", "yParam", 0, 0.1, 0);		
			
			jf.add(pg, 1,0);
			jf.add(rgbJim, 0, 0);
			jf.add(depthJim, 0, 1);
			jf.setSize(1280,960);
		}
		else if(colorAnalyze2)
		{
			jf.setLayout(new GridLayout(2,2));
			pg = new ParameterGUI();
			pg.addDoubleSlider("HueMin","Hue Min",0,1.0,0);
			pg.addDoubleSlider("HueMax","Hue Max",0,1.0,.2);
			pg.addDoubleSlider("SatMin","Saturation Min",0,1.0,0);
			pg.addDoubleSlider("SatMax","Saturation Max",0,1.0,1.0);
			pg.addDoubleSlider("BrightMin","Brightness Min",0,1.0,0);
			pg.addDoubleSlider("BrightMax","Brightness Max",0,1.0,1.0);
			pg.addDoubleSlider("xParam", "xParam", 0d, 1.0, 0);	
			pg.addDoubleSlider("yParam", "yParam", 0d, 1.0, 0);
	        pg.addListener(new ParameterListener() {
	            public void parameterChanged(ParameterGUI _pg, String name)
	            {
	                if (name.equals("xParam")) {
	                	x_param = _pg.gd(name);
	                }
	                else if (name.equals("yParam")) {
	                	y_param = _pg.gd(name);
	                }
	            }
	        });

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

	public static void main(String[] args)
	{
		final KinectView kv = new KinectView(true);
		kv.kinect.setDepthFormat(DepthFormat.D11BIT);
		double params[] = new double[6];
		if(kv.colorAnalyze)
		{
			params[0] = (double)kv.pg.gi("redValMin");
			params[2] = (double)kv.pg.gi("greenValMin");
			params[4] = (double)kv.pg.gi("blueValMin");
			params[1] = (double)kv.pg.gi("redValMax");
			params[3] = (double)kv.pg.gi("greenValMax");
			params[5] = (double)kv.pg.gi("blueValMax");
		}
		else if(kv.colorAnalyze2)
		{
			params[0] = kv.pg.gd("HueMin");
			params[1] = kv.pg.gd("HueMax");
			params[2] = kv.pg.gd("SatMin");
			params[3] = kv.pg.gd("SatMax");
			params[4] = kv.pg.gd("BrightMin");
			params[5] = kv.pg.gd("BrightMax");
		}
		BallTracker tracker = new BallTracker(kv.width,kv.height,true);

		KinectRGBVideo kcv = new KinectRGBVideo(kv.kinect,1,params);
		KinectDepthVideo kdv = new KinectDepthVideo(kv.kinect);

		kv.depthImg = kdv.getFrame();
		kv.rgbImg = kcv.getFrame();
		while(true) 
		{
			while(!kcv.newImage && !kdv.newImage);
			ball_t ballLCM = new ball_t();
			ballLCM.nanoTime = kv.globalTime;
			kcv.newImage = false;
			kdv.newImage = false;
			Point poi = new Point();
			poi.x = 320;
			poi.y = 240;
			ArrayList<Statistics> blobs = tracker.analyzePartition(kcv.getValidImage(), poi, 640,480, "center");
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
			Statistics ball = BiggestBlob;
			// if(kv.tracking)
			// {
				// for(Statistics ball : kv.trajectory)
				// {
					for(int y = ball.center_y-3; y < ball.center_y+3; y++)
						for(int x = ball.center_x-3; x < ball.center_x+3;x++)
							try{
								kv.rgbImg.setRGB(x,y,0xFFFF0000);
			//draw bounding box to determine if ball will fall in place
								int depthy = kv.intoDepthY(y);
								int depthx = kv.intoDepthX(x);
								kv.depthImg.setRGB(depthx, depthy, 0xFFFFFFFF);
								// kv.depthImg.setRGB(x,y,0xFFFFFFFF);
							}
							catch(Exception e){};	
				// }
			// }
			// else
			// {
			// 	kv.trajectory.clear();
			// }
						//draw bounding box to determine if ball will fall in place
			// try
			// {
			// 	int bound = 160;
			// 	for(int y = kv.BALL.center_y-(bound/2); y < kv.BALL.center_y+(bound/2); y++)
			// 	{
			// 		kv.depthImg.setRGB(kv.BALL.center_x-(bound/2),y,0xFFFFFFFF);
			// 		kv.depthImg.setRGB(kv.BALL.center_x+(bound/2),y,0xFFFFFFFF);
			// 	}
			// 	for(int x = kv.BALL.center_x-(bound/2); x < kv.BALL.center_x+(bound/2); x++)
			// 	{
			// 		kv.depthImg.setRGB(x,kv.BALL.center_y-(bound/2),0xFFFFFFFF);
			// 		kv.depthImg.setRGB(x,kv.BALL.center_y+(bound/2),0xFFFFFFFF);
			// 	}
			// }
			// catch(Exception e){};
				//System.println(kv.getDepth(kv.depthImg,kv.BALL.center_y*width+kv.BALL.center_x));

				ballLCM.x = kv.BALL.center_x;
				ballLCM.y = kv.BALL.center_y;
				ballLCM.z = 4;
				kv.lcm.publish("6_BALL",ballLCM);
			// Point ballCenter = new Point(kv.BALL.center_x,kv.BALL.center_y);
			// ballCenter.x = ball.x;
			// ballCenter.y = ball.y;
			// ArrayList<Statistics> depthBlobs = tracker.analyzeDepthPartition(kdv.getBuf(),ballCenter,150);
			// Statistics ClosestBall = new Statistics();
			// for(Statistics blob : depthBlobs)
			// {
			// 	if(blob.closestDepth < ClosestBall.closestDepth)
			// 	{
			// 		ClosestBall = blob;
			// 	}
			// }
			// if(ClosestBall.closestPixel != null)
			// {
			// 	for(int y = ClosestBall.closestPixel.y-3; y < ClosestBall.closestPixel.y+3; y++)
			// 			for(int x = ClosestBall.closestPixel.x-3; x < ClosestBall.closestPixel.x+3;x++)
			// 				try{
			// 					kv.depthImg.setRGB(x,y,0xFFFFFFFF);
			// 					// int depthx = intoDepthY(y);
			// 					// int depthy = intoDepthX(x);
			// 					// kv.depthImg.setRGB(depthx, depthy, 0xFFFFFFFF);
			// 					//kv.depthImg.setRGB(x,y,0xFFFFFFFF);
			// 				}
			// 				catch(Exception e){};
			// 	System.out.println(ClosestBall.closestDepth);
			// }
			
			kv.rgbJim.setImage(kv.rgbImg);
			kv.depthJim.setImage(kv.depthImg);
			if(kv.colorAnalyze)
			{
				params[0] = (double)kv.pg.gi("redValMin");
				params[2] = (double)kv.pg.gi("greenValMin");
				params[4] = (double)kv.pg.gi("blueValMin");
				params[1] = (double)kv.pg.gi("redValMax");
				params[3] = (double)kv.pg.gi("greenValMax");
				params[5] = (double)kv.pg.gi("blueValMax");
				kcv.changeThreshold((byte)2,params);
			}
			else if(kv.colorAnalyze2)
			{
				params[0] = kv.pg.gd("HueMin");
				params[1] = kv.pg.gd("HueMax");
				params[2] = kv.pg.gd("SatMin");
				params[3] = kv.pg.gd("SatMax");
				params[4] = kv.pg.gd("BrightMin");
				params[5] = kv.pg.gd("BrightMax");
				kcv.changeThreshold((byte)1,params);
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
	public int getDepth(ByteBuffer bb, int index) {
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
