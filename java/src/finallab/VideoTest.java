package finallab;

import javax.swing.JFrame;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;

import finallab.CalibrateCam.WindowClose;

public class VideoTest {

	Context ctx;
	Device kinect;
	JFrame windowRGB;
	JFrame windowDepth;
	KinectRGBVideo rgbVideo;
	KinectDepthVideo depthVideo;
	
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
		depthVideo = new KinectDepthVideo(kinect);
		windowDepth.setSize(640, 480);
		windowDepth.setContentPane(depthVideo);
		windowDepth.setVisible(true);
	}
	public static void main(String[] args) {
		VideoTest vt = new VideoTest();
		

	}

}
