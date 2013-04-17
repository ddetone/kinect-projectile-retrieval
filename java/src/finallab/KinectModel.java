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

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;

import finallab.lcmtypes.*;

import lcm.lcm.*;

public class KinectModel
{

	JFrame jf = new JFrame("KinectModel");

	VisWorld vw = new VisWorld();
	VisLayer vl = new VisLayer(vw);
	VisCanvas vc = new VisCanvas(vl);

	KinectDepthVideo kdv;
	KinectRGBVideo krv;

	Context ctx;
	Device kinect;

	public KinectModel() {
		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
		}

		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);

		jf.setSize(800,600);
		jf.setVisible(true);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		vl.cameraManager.uiLookAt(new double[] {-2.66075, 1.22066, 1.70393 },
				new double[] {1.75367, -0.06226,  0.00000 },
				new double[] {0.33377, -0.09695,  0.93766 }, true);

		kdv = new KinectDepthVideo(kinect, null, false);
		krv = new KinectRGBVideo(kinect, null, true);
		JFrame rgbFrame = new JFrame("rgb feed");
		rgbFrame.setContentPane(krv);
		rgbFrame.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		rgbFrame.setVisible(true);
		
	}

	public static void drawWorld(KinectModel km)
	{
		// VisChain env = new VisChain();
		VisChain world = new VisChain();
		ArrayList<double[]> worldPoints = new ArrayList<double[]>();
		for (int i = 0; i < KinectVideo.WIDTH * KinectVideo.HEIGHT; i += 1) {
			Point currCoord = new Point();
			currCoord.x = (i % KinectVideo.WIDTH) - KinectVideo.C_X;
			currCoord.y = KinectVideo.C_Y - (i / KinectVideo.WIDTH);
			Point3D worldPoint = km.kdv.getWorldCoords(currCoord);
			if (!(worldPoint.x == 0 && worldPoint.y == 0 && worldPoint.z == 0) && worldPoint.z < 3) {
				double [] point = {worldPoint.x, worldPoint.y, worldPoint.z};
//				System.out.println("x:" + worldPoint.x + " y:" + worldPoint.y + " z:" + worldPoint.z);
				worldPoints.add(point);
			}
		}
//		km.kdv.stop();
		VisWorld.Buffer vb = km.vw.getBuffer("Environment");
		vb.addBack(new VzPoints(new VisVertexData(worldPoints), new VzPoints.Style(Color.BLUE, 2)));
		vb.swap();
			System.out.println("plotting");

	}

	public static void main(String[] args) throws Exception
	{
		KinectModel km = new KinectModel();
		 while(true)
		 {

//			Thread.sleep(2000);
			drawWorld(km);
		 }


	}
}