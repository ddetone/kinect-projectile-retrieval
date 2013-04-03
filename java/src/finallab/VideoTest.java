package finallab;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;

import april.util.JImage;

import finallab.CalibrateCam.WindowClose;
import finallab.lcmtypes.ball_t;

public class VideoTest {

	Context ctx;
	Device kinect;
	JFrame windowRGB;
	JFrame windowDepth;
	KinectRGBVideo rgbVideo;
	KinectDepthVideo depthVideo;
	
	JFrame unionFindWindow;
	JImage ballJim;
	BufferedImage ballImage;
	boolean newImage = false;
	Statistics BALL;
	
	public VideoTest() {
		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
		}
		
		
		windowRGB = new JFrame("RGB camera");
		windowRGB.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		rgbVideo = new KinectRGBVideo(kinect);
		windowRGB.setSize(640, 480);
		windowRGB.setContentPane(rgbVideo);
		windowRGB.setVisible(true);
		
		windowDepth = new JFrame("Depth camera");
		windowDepth.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		
		ballImage = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		ballJim = new JImage();
		ballJim.setImage(ballImage);
		unionFindWindow = new JFrame("union find");
		windowRGB.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		unionFindWindow.setSize(640, 480);
		unionFindWindow.add(ballJim);
		unionFindWindow.setVisible(true);
		
	}
	public static void main(String[] args) {
		VideoTest vt = new VideoTest();
		depthUnionFindTest(vt);
		

	}
	public static void depthUnionFindTest(VideoTest vt) {
		vt.depthVideo = new KinectDepthVideo(vt.kinect, vt);
		vt.windowDepth.setSize(640, 480);
		vt.windowDepth.setContentPane(vt.depthVideo);
		vt.windowDepth.setVisible(true);
		BallTracker tracker = new BallTracker(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		while(vt.depthVideo.getBuf() == null);
		while(true) 
		{
//			while(!vt.newImage);
			vt.newImage = false;
			ArrayList<Statistics> blobs = tracker.analyzeDepth(vt.depthVideo.getBuf());
			Statistics BiggestBlob = new Statistics();
			for(Statistics blob : blobs)
			{
				if(blob.N > 10)
				{
					// if(kv.BALL.Cxy() > blob.Cxy());
						// if(kv.BALL.abs() > blob.abs())
							// kv.BALL = blob;
					if(BiggestBlob.N < blob.N)
						BiggestBlob = blob;
					blob.center();
				}
			}
			System.out.println("num blobs: " + blobs.size());
			vt.BALL = BiggestBlob;
			vt.BALL.center();
			for(int y = vt.BALL.center_y-3; y < vt.BALL.center_y+3; y++) {
				for(int x = vt.BALL.center_x-3; x < vt.BALL.center_x+3;x++) {
					try{
						vt.ballImage.setRGB(x,y,0xFFFF0000);
						// int depthx = intoDepthY(y);
						// int depthy = intoDepthX(x);
						// kv.depthImg.setRGB(depthx, depthy, 0xFFFFFFFF);
					}
					catch(Exception e){};
				}
			}
		

			vt.ballJim.setImage(vt.ballImage);
			// try {
			// 	Thread.sleep(100);
			// }
			// catch(Exception e) {
				
		}
	}

}
