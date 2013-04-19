package finallab;

import java.nio.ByteBuffer;
import java.awt.Color;
import java.awt.Point;

import org.openkinect.freenect.Device;
import org.openkinect.freenect.FrameMode;
import org.openkinect.freenect.VideoHandler;

public class KinectRGBVideo extends KinectVideo 
{	
	private static final long serialVersionUID = 1L;
	
	Boolean [] validImageValue;
	byte colorAnalyzeMode;
	double hueMin,hueMax,satMin,satMax,brightMin,brightMax;
	int redMin, redMax, greenMin, greenMax, blueMin, blueMax;

	public KinectRGBVideo(Device kinect, Object _imgMonitor, boolean _display) {
		super(kinect, _imgMonitor, _display);
		
		f = 527.273;
		kinect.startVideo(new VideoHandler() {
			@Override
			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int _timestamp) {
				
				frameData = rgb;
				timestamp = _timestamp; 
				if (display) {
					frameLock.writeLock().lock();
					try {
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
						repaint();
					}
					finally {
						frameLock.writeLock().unlock();
					}
				}
			}
		});
	}
	// public KinectRGBVideo(Device kinect, int colorAnalyzeMode, double[] params) 
	// {
	// 	super(kinect,true);
	// 	f = 527.273;
	// 	if((colorAnalyzeMode>2)||(colorAnalyzeMode == 0))
	// 	{

	// 		kinect.startVideo(new VideoHandler() {
	// 			@Override
	// 			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int _timestamp) {
	// 				frameData = rgb;
	// 				timestamp = _timestamp; 
	// 				// if (!display)
	// 				// 	return;
	// 				int[] pixelInts = new int[WIDTH * HEIGHT];
	// 				for(int i = 0; i < WIDTH*HEIGHT; i++) {
	// 					int rgbVal = 0xFF;
	// 					for(int j = 0; j < 3; j++) {
	// 						int data = rgb.get() & 0xFF;
	// 						rgbVal = rgbVal << 8;
	// 						rgbVal = rgbVal | (data);
	// 					}
	// 					pixelInts[i] = rgbVal;
	// 				}
	// 				frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
	// 				rgb.position(0);
	// 				// repaint();
	// 				newImage = true;
	// 			}
	// 		});
	// 	}
	// 	else if(colorAnalyzeMode == 1)
	// 	{
	// 		if(params == null)
	// 		{
	// 			System.err.println("Did not specify rgb threshold parameters.");
	// 			System.exit(1);
	// 		}
	// 		hueMin = params[0];
	// 		hueMax = params[1];
	// 		satMin = params[2];
	// 		satMax = params[3];
	// 		brightMin = params[4];
	// 		brightMax = params[5];
	// 		validImageValue = new Boolean[WIDTH*HEIGHT];
	// 		kinect.startVideo(new VideoHandler() {
	// 			@Override
	// 			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int _timestamp) {
	// 				frameData = rgb;
	// 				timestamp = _timestamp; 
	// 				if (!display)
	// 					return;
	// 				int[] pixelInts = new int[WIDTH * HEIGHT];

	// 				int red = 0;
	// 				int green = 0;
	// 				int blue = 0;

	// 				for(int i = 0; i < WIDTH*HEIGHT; i++) {
	// 					int rgbVal = 0xFF;
	// 					for(int j = 0; j < 3; j++) {
	// 						int data = rgb.get() & 0xFF;
	// 						rgbVal = rgbVal << 8;
	// 						rgbVal = rgbVal | (data);

	// 						if(j == 0)
	// 							red = data;
	// 						else if(j==1)
	// 							green = data;
	// 						else
	// 							blue = data;
	// 					}
	// 					pixelInts[i] = rgbVal;
	// 					float hsb[] = Color.RGBtoHSB(red,green,blue,null);
	// 					if(hueMin >= hsb[0] || hueMax <= hsb[0] || satMin >=  hsb[1] || satMax <=  hsb[1]|| brightMin >=  hsb[2]|| brightMax <= hsb[2])
	// 					{
	// 						pixelInts[i] = 0xFFFFFFFF;
	// 						validImageValue[i] = false;
	// 					}
	// 					else
	// 					{
	// 						validImageValue[i] = true;
	// 					}
	// 				}
	// 				frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
	// 				rgb.position(0);
	// 				newImage = true;
	// 			}	
	// 		});
	// 	}
	// 	else if(colorAnalyzeMode == 2)
	// 	{
	// 		if(params == null)
	// 		{
	// 			System.err.println("Did not specify rgb threshold parameters.");
	// 			System.exit(1);
	// 		}
	// 		redMin = (int)params[0];
	// 		redMax = (int)params[1];
	// 		blueMin = (int)params[2];
	// 		blueMax = (int)params[3];
	// 		greenMin = (int)params[4];
	// 		greenMax = (int)params[5];
	// 		validImageValue = new Boolean[WIDTH*HEIGHT];
	// 		kinect.startVideo(new VideoHandler() {
	// 			@Override
	// 			public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int _timestamp) {
	// 				frameData = rgb;
	// 				timestamp = _timestamp; 
	// 				if (!display)
	// 					return;
	// 				int[] pixelInts = new int[WIDTH * HEIGHT];

	// 				int red = 0;
	// 				int green = 0;
	// 				int blue = 0;

	// 				for(int i = 0; i < WIDTH*HEIGHT; i++) {
	// 					int rgbVal = 0xFF;
	// 					for(int j = 0; j < 3; j++) {
	// 						int data = rgb.get() & 0xFF;
	// 						rgbVal = rgbVal << 8;
	// 						rgbVal = rgbVal | (data);

	// 						if(j == 0)
	// 							red = data;
	// 						else if(j==1)
	// 							green = data;
	// 						else
	// 							blue = data;
	// 					}
	// 					pixelInts[i] = rgbVal;

	// 					if(redMin >= red || redMax <= red || greenMin >= green || greenMax <= green || blueMin >= blue || blueMax <= blue)
	// 					{
	// 						pixelInts[i] = 0xFFFFFFFF;
	// 						validImageValue[i] = false;
	// 					}
	// 					else
	// 					{
	// 						validImageValue[i] = true;
	// 					}
	// 				}
	// 				frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
	// 				rgb.position(0);	
	// 				newImage = true;
	// 			}	
	// 		});
	// 	}
	// }

	// public void changeThreshold(byte analyzeMode, double[] params)
	// {
	// 	if(analyzeMode == 1)
	// 	{
	// 		hueMin = params[0];
	// 		hueMax = params[1];
	// 		satMin = params[2];
	// 		satMax = params[3];
	// 		brightMin = params[4];
	// 		brightMax = params[5];
	// 	}
	// 	else if(analyzeMode == 2)
	// 	{	
	// 		redMin = (int)params[0];
	// 		redMax = (int)params[1];
	// 		blueMin = (int)params[2];
	// 		blueMax = (int)params[3];
	// 		greenMin = (int)params[4];
	// 		greenMax = (int)params[5];
	// 	}
	// }	

	public Boolean[] getValidImage()
	{
		return validImageValue;
	}	
	//tennis ball
	// public double guessDepth(int xDiff) {
	// 	if (xDiff >= 46) {
	// 		return .8;
	// 	}
	// 	else if (xDiff >= 37 && xDiff < 46) {
	// 		return 1.0;
	// 	}
	// 	else if (xDiff >= 32 && xDiff < 37) {
	// 		return 1.2;
	// 	}
	// 	else if (xDiff >= 28 && xDiff < 32) {
	// 		return 1.4;
	// 	}
	// 	else if (xDiff >= 24 && xDiff < 28) {
	// 		return 1.6;
	// 	}
	// 	else if (xDiff >= 22 && xDiff < 24) {
	// 		return 1.8;
	// 	}
	// 	else if (xDiff >= 19 && xDiff < 22) {
	// 		return 2.0;
	// 	}
	// 	else if (xDiff >= 17 && xDiff < 19) {
	// 		return 2.2;
	// 	}
	// 	else if (xDiff >= 15 && xDiff < 17) {
	// 		return 2.4;
	// 	}
	// 	else if (xDiff >= 14 && xDiff < 15) {
	// 		return 2.6;
	// 	}
	// 	else if (xDiff >= 12 && xDiff < 14) {
	// 		return 2.8;
	// 	}
	// 	else {
	// 		return 3.0;
	// 	}

	// }

	//sky ball
	public double guessDepth(int xDiff) {
		if (xDiff >= 58) {
			return .8;
		}
		else if (xDiff >= 47) {
			return 1.0;
		}
		else if (xDiff >= 39) {
			return 1.2;
		}
		else if (xDiff >= 33) {
			return 1.4;
		}
		else if (xDiff >= 29) {
			return 1.6;
		}
		else if (xDiff >= 27) {
			return 1.8;
		}
		else if (xDiff >= 24) {
			return 2.0;
		}
		else if (xDiff >= 21) {
			return 2.2;
		}
		else if (xDiff >= 19) {
			return 2.4;
		}
		else if (xDiff >= 18) {
			return 2.6;
		}
		else if (xDiff >= 17) {
			return 2.8;
		}
		else {
			return 3.0;
		}

	}
}
