package finallab;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;
import april.jmat.geom.*;

import finallab.lcmtypes.*;

import lcm.lcm.*;

public class KinectView implements LCMSubscriber
{

	JFrame jf;
	JImage rgbImage;
	JImage depthImage;

	LCM lcm;

	final boolean verbose = false;

	KinectView()
	{
		try{
			this.lcm = new LCM();
		}catch(IOException e){
			this.lcm = LCM.getSingleton();
		}
		lcm.subscribe("KINECT_STATUS",this);
		jf = new JFrame("KinectView");
		rgbImage = new JImage();
		depthImage = new JImage();		

		jf.setLayout(new GridLayout(2,1));

		jf.add(rgbImage, BorderLayout.EAST);
		jf.add(depthImage, BorderLayout.WEST);

		jf.setSize(1600,600);
		jf.setVisible(true);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public static void main(String[] args)
	{
		KinectView kv = new KinectView();
		while(true);
	}

	public void updateImages(byte[] rgbData, byte[] depthData)
	{
		BufferedImage rgbIM = null;
		BufferedImage depthIM = null;
		DataBufferByte dataBuffer = new DataBufferByte(rgbData, 640*480);
		Raster raster = Raster.createPackedRaster(dataBuffer, 640, 480, 8, null);
		rgbIM = new BufferedImage(640, 480, BufferedImage.TYPE_BYTE_GRAY);
		rgbIM.setData(raster);
	
		// if(rgbData != null)
  //       {
  //           // Grab the image, and convert it to gray scale immediately
  //        	rgbIM = ImageConvert.convertToImage(fmt.format, 640, 480, rgbData);
		// }
		// if(depthData != null)
		// {
		// 	depthIM = ImageConvert.convertToImage(fmt.format, 640, 480, depthData);
		// }
		rgbImage.setImage(rgbIM);
		//depthImage.setImage(depthIM);

	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("KINECT_STATUS"))
			{
				kinect_status_t kinectData = new kinect_status_t(dins);
				System.out.println("Got Kinect Image Data");
				updateImages(kinectData.rgb,kinectData.depth);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

}