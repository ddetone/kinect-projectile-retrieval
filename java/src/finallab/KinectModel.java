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

		vl.cameraManager.uiLookAt(
		          new double[] {1, 1, 1 },
		          new double[] { 0,  0, 0.00000 },
		          new double[] { 0.13802,  0.40084, 0.90569 }, true);

		kdv = new KinectDepthVideo(kinect, false);
	}

	public static void drawWorld(KinectModel km)
	{
		// VisChain env = new VisChain();
		VisWorld.Buffer vb = km.vw.getBuffer("Environment");
		for (int i = 0; i < KinectVideo.WIDTH * KinectVideo.HEIGHT; i += 10) {
			Point currCoord = new Point();
			currCoord.x = (i % KinectVideo.WIDTH) - KinectVideo.C_X;
			currCoord.y = KinectVideo.C_Y - (i / KinectVideo.WIDTH);
			// System.out.println("plotting " + world.x + " " + world.y + " " + world.z);
			Point3D world = km.kdv.getWorldCoords(currCoord);
			vb.addBack(new VisChain(LinAlg.translate(world.x, world.y, world.z), new VzSphere(0.01, new VzMesh.Style(Color.pink))));
		}
			System.out.println("plotting");

		// vb.addBack(env);
		vb.swap();

	}

	public static void main(String[] args) throws Exception
	{
		KinectModel km = new KinectModel();
		// while(true)
		// {

			Thread.sleep(2000);
			drawWorld(km);
		// }


	}
}