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
	
//	final static short width = 640;
//	final static short height = 480;
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

	KinectView()
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
	}

	// public void publishBall(int timestamp)
	// {

	// 	ball_t ball = new ball_t();
	// 	ball.nanoTime = timestamp;
	// 	ball.x = BALL.center_x;
	// 	ball.y = BALL.center_y;
	// 	// ball.z = ballDepth;
	// 	lcm.publish("6_BALL",ball);

	// }

	public static void main(String[] args)
	{
		final KinectView kv = new KinectView();
		kv.kinect.setDepthFormat(DepthFormat.D11BIT);
		
		kv.kinect.startVideo(new VideoHandler() {
			
			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int timestamp) {
				kv.globalTime = System.nanoTime();
				kv.bufToRGBImage(fm, rgb, timestamp);
				// kv.rgbJim.setImage(kv.rgbImg);
			}
			
		});
		kv.kinect.setDepthFormat(DepthFormat.D11BIT);
		// kv.kinect.startDepth(new DepthHandler() {

		// 	@Override
		// 	public void onFrameReceived(FrameMode fm, ByteBuffer depth, int timestamp) {
		// 		kv.bufToDepthImage(fm, depth);
		// 		// kv.depthJim.setImage(kv.depthImg);
		// 	}
			
		// });
		BallTracker tracker = new BallTracker(kv.width,kv.height,true);
		KinectDepthVideo kdv = new KinectDepthVideo(kv.kinect, true);
		kv.depthImg = kdv.getFrame();
		while(true) 
		{
			while(!kv.newImage && !kdv.newImage);
			ball_t ballLCM = new ball_t();
			ballLCM.nanoTime = kv.globalTime;
			kv.newImage = false;
			kdv.newImage = false;
			Point poi = new Point();
			poi.x = 320;
			poi.y = 240;
			ArrayList<Statistics> blobs = tracker.analyzePartition(kv.validImageValue, poi, 640,480, "center");
			Statistics BiggestBlob = new Statistics();
			for(Statistics blob : blobs)
			{
				if(blob.N > 50)
				{
					// if(kv.BALL.Cxy() > blob.Cxy());
						// if(kv.BALL.abs() > blob.abs())
							// kv.BALL = blob;
					if(BiggestBlob.N < blob.N)
						BiggestBlob = blob;
					blob.center();
				}
				kv.BALL = BiggestBlob;
				kv.BALL.center();
				/*for(int y = kv.BALL.center_y-3; y < kv.BALL.center_y+3; y++)
					for(int x = kv.BALL.center_x-3; x < kv.BALL.center_x+3;x++)
						try{
							kv.rgbImg.setRGB(x,y,0xFFFF0000);
							// int depthx = intoDepthY(y);
							// int depthy = intoDepthX(x);
							// kv.depthImg.setRGB(depthx, depthy, 0xFFFFFFFF);
						}
						catch(Exception e){};*/
				
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
			// try {
			// 	Thread.sleep(100);
			// }
			// catch(Exception e) {
				
		}
	}
	
	/*
	 *  this creates the image by iterating through each byte...
	 *  we can just use pixelsRGB.get(index) in practice
	 *  
	 *  ByteBuffer is 3 * width * height
	 *  3 bytes per pixel (RGB)
	 */
	private void bufToRGBImage(FrameMode fm, ByteBuffer rgb, int timestamp) {
		width = fm.getWidth();
		height = fm.getHeight();
		int[] pixelInts = new int[width * height];
		int redMin;
		int greenMin;
		int blueMin;
		int redMax;
		int greenMax;
		int blueMax;
		double hueMin;
		double hueMax;
		double brightMin;
		double brightMax;
		double satMin;
		double satMax;
		if(colorAnalyze)
		{
			redMin = pg.gi("redValMin");
			greenMin = pg.gi("greenValMin");
			blueMin = pg.gi("blueValMin");
			redMax = pg.gi("redValMax");
			greenMax = pg.gi("greenValMax");
			blueMax = pg.gi("blueValMax");
		}
		else if(colorAnalyze2)
		{
			hueMin = pg.gd("HueMin");
			hueMax = pg.gd("HueMax");
			satMin = pg.gd("SatMin");
			satMax = pg.gd("SatMax");
			brightMin = pg.gd("BrightMin");
			brightMax = pg.gd("BrightMax");
		}

		int red = 0;
		int green = 0;
		int blue = 0;

		for(int i = 0; i < width*height; i++) {
			int rgbVal = 0xFF;
			for(int j = 0; j < 3; j++) {
				int data = rgb.get() & 0xFF;
				rgbVal = rgbVal << 8;
				rgbVal = rgbVal | (data);

				if(j == 0)
					red = data;
				else if(j==1)
					green = data;
				else
					blue = data;
			}
			pixelInts[i] = rgbVal;
			if(colorAnalyze)
			{

				if(redMin >= red || redMax <= red || greenMin >= green || greenMax <= green || blueMin >= blue || blueMax <= blue)
				{
					pixelInts[i] = 0xFFFFFFFF;
					validImageValue[i] = false;
				}
				else
				{
					validImageValue[i] = true;
				}
			}
			else if(colorAnalyze2)
			{
				float hsb[] = Color.RGBtoHSB(red,green,blue,null);
				if(hueMin >= hsb[0] || hueMax <= hsb[0] || satMin >=  hsb[1] || satMax <=  hsb[1]|| brightMin >=  hsb[2]|| brightMax <= hsb[2])
				{
					pixelInts[i] = 0xFFFFFFFF;
					validImageValue[i] = false;
				}
				else
				{
					validImageValue[i] = true;
				}
			}	
		}

		

		//set position to 0 because ByteBuffer is reused to access byte array of new frame
		//and get() below increments the iterator



		// publishBall(timestamp);


		rgbImg.setRGB(0, 0, width, height, pixelInts, 0, width);
		newImage = true;

		//create ball tracker with map of valid ball pixels


		//color center of image white
		// for(int y = (height/2)-5; y < (height/2)+5; y++)
		// 	for(int x = (width/2)-5; x < (width/2)+5;x++)
		// 		pixelInts[y*width+x] = 0xFFFFFFFF;

		//get slider values for r g & b



		rgb.position(0);
		
	}
	
	/*  
	 *  ByteBuffer is 2 * width * height
	 *  10 bit number for distance for each pixel
	 *  first byte is LSB 7-0
	 *  second byte is MSB 1-0
	 */
	private void bufToDepthImage(FrameMode fm, ByteBuffer depthBuf) {
		width = fm.getWidth();
		height = fm.getHeight();
		int[] pixelInts = new int[width * height];

		//ballDepth = getDepth(depthBuf,BALL.center_x*width + BALL.center_y);

		for(int i = 0; i < width*height; i++) {
			
			int depth = 0;
			byte byte1 = depthBuf.get();
			byte byte2 = depthBuf.get();
			depth = byte2 & 0x7;
			depth = depth << 8;
			depth = depth | (byte1 & 0xFF);

			if (i == ((height /2) * width + width/2)) {
			 	//System.out.println("Array depth: "+ depth);
			 	//System.out.println("Estimate Meter Depth: " + raw_depth_to_meters(depth));
			}
			/*
			 * color scaled depth
			 * closest -> farthest
			 * black -> red -> yellow -> green
			 * (depth starts registering ~390 or ~ .5m)
			 */
			int r = 0x0000;
			int g = 0x0000;
			int b = 0x0000;
			depth = Math.abs(depth - 390);
			int interval = (1023 - 350) / 3;
			double step = 255.0 / (double)interval;
			if (depth <= (interval)) {				
				r = (int)(depth * step) & 0xFF;
			}
			else if (depth <= (2*interval)) {
				r = 0xFF;
				g = (int)((depth - interval) * step) & 0xFF;
			}
			else if (depth <= (3*interval)) {
				r = (int)((interval - (depth % interval)) * step) & 0xFF;
				g = 0xFF;
			}
			else {
				g = 0xFF;
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
		//draw box at center of image
		// for(int y = (height/2)-5; y < (height/2)+5; y++)
		// 	for(int x = (width/2)-5; x < (width/2)+5;x++)
		// 		pixelInts[y*width+x] = 0xFFFFFFFF;

		depthImg.setRGB(0, 0, width, height, pixelInts, 0, width);
		
		//set position to 0 because ByteBuffer is reused to access byte array of new frame
		//and get() below increments the iterator
		depthBuf.position(0);
		
	}
	
	public float raw_depth_to_meters(int raw_depth)
	{
 		if (raw_depth < 2047)
  		{
   			return (1.0f / (raw_depth * -0.0030711016f + 3.3309495161f))+.05f;
  		}
  		return 0;
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
	*/

	
	//utility functions to copy/paste later	
	public int getDepth(ByteBuffer bb, int index) {
		int depth = 0;
		byte byte1 = bb.get(index * 2);
		byte byte2 = bb.get(index * 2 + 1);
		depth = byte2 & 0x7;
		depth = depth << 8;
		depth = depth | (byte1 & 0xFF);
		return depth & 0x7FF;
	}
	

}