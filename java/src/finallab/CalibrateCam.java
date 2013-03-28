package finallab;

import java.util.*;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.*;

import javax.swing.JFrame;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;

import april.util.*;
import april.jmat.*;

public class CalibrateCam
{
	Context ctx;
	Device kinect;
	JFrame window;
	KinectRGBVideo rgbVideo;
	
	CalibrateCam()
	{
		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
		}
		window = new JFrame("calibrate camera");
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		rgbVideo = new KinectRGBVideo(kinect);
		window.setContentPane(rgbVideo);
	}
	
	MouseListener calibrateTheta = new MouseAdapter() {
		public void mouseClicked(MouseEvent e) {
			Point p = e.getPoint();
			System.out.println("x: " + p.x + ", y: " + p.y);
			System.out.println("x dist: " + (p.x - KinectRGBVideo.WIDTH) + ", y dist: " + (p.y - KinectRGBVideo.HEIGHT));
		}
	};
	public static void main(String [] args) {
		CalibrateCam cc = new CalibrateCam();
	}
	
	
	


}
