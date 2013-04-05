package finallab;

import java.nio.ByteBuffer;
import java.awt.Color;

import org.openkinect.freenect.Device;
import org.openkinect.freenect.FrameMode;
import org.openkinect.freenect.VideoHandler;

public class KinectRGBVideo extends KinectVideo 
{	
	private static final long serialVersionUID = 1L;
	public volatile boolean newImage = false;
	
	Boolean [] validImageValue;
	byte colorAnalyzeMode;
	double hueMin,hueMax,satMin,satMax,brightMin,brightMax;
	int redMin, redMax, greenMin, greenMax, blueMin, blueMax;

	public KinectRGBVideo(Device kinect, int colorAnalyzeMode, double[] params) 
	{
		super(kinect);
		if((colorAnalyzeMode>2)||(colorAnalyzeMode == 0))
		{

			kinect.startVideo(new VideoHandler() {
				@Override
				public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int timestamp) {
					frameData = rgb;
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
				//repaint();
					newImage = true;
				}
			});
		}
		else if(colorAnalyzeMode == 1)
		{
			if(params == null)
			{
				System.err.println("Did not specify rgb threshold parameters.");
				System.exit(1);
			}
			hueMin = params[0];
			hueMax = params[1];
			satMin = params[2];
			satMax = params[3];
			brightMin = params[4];
			brightMax = params[5];
			validImageValue = new Boolean[WIDTH*HEIGHT];
			kinect.startVideo(new VideoHandler() {
				@Override
				public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int timestamp) {
					frameData = rgb;
					int[] pixelInts = new int[WIDTH * HEIGHT];

					int red = 0;
					int green = 0;
					int blue = 0;

					for(int i = 0; i < WIDTH*HEIGHT; i++) {
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
					frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
					rgb.position(0);
					newImage = true;
				}	
			});
		}
		else if(colorAnalyzeMode == 2)
		{
			if(params == null)
			{
				System.err.println("Did not specify rgb threshold parameters.");
				System.exit(1);
			}
			redMin = (int)params[0];
			redMax = (int)params[1];
			blueMin = (int)params[2];
			blueMax = (int)params[3];
			greenMin = (int)params[4];
			greenMax = (int)params[5];
			validImageValue = new Boolean[WIDTH*HEIGHT];
			kinect.startVideo(new VideoHandler() {
				@Override
				public void onFrameReceived(FrameMode fm, ByteBuffer rgb, int timestamp) {
					frameData = rgb;
					int[] pixelInts = new int[WIDTH * HEIGHT];

					int red = 0;
					int green = 0;
					int blue = 0;

					for(int i = 0; i < WIDTH*HEIGHT; i++) {
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
					frame.setRGB(0, 0, WIDTH, HEIGHT, pixelInts, 0, WIDTH);
					rgb.position(0);	
					newImage = true;
				}	
			});
		}
	}

	public void changeThreshold(byte analyzeMode, double[] params)
	{
		if(analyzeMode == 1)
		{
			hueMin = params[0];
			hueMax = params[1];
			satMin = params[2];
			satMax = params[3];
			brightMin = params[4];
			brightMax = params[5];
		}
		else if(analyzeMode == 2)
		{	
			redMin = (int)params[0];
			redMax = (int)params[1];
			blueMin = (int)params[2];
			blueMax = (int)params[3];
			greenMin = (int)params[4];
			greenMax = (int)params[5];
		}
	}	

	public Boolean[] getValidImage()
	{
		return validImageValue;
	}	
}
