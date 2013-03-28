package finallab;

import java.util.*;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.*;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

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
	
	final static int C_X = KinectRGBVideo.WIDTH / 2;
	final static int C_Y = KinectRGBVideo.HEIGHT / 2;
	
	ArrayList<Integer> arr_p_x;
	ArrayList<Integer> arr_p_y;
	ArrayList<Double[]> arr_r_x;
	ArrayList<Double[]> arr_r_y;
	
	Matrix r;
	Matrix p;
	CalibrateCam()
	{
		arr_p_x = new ArrayList<Integer>();
		arr_p_y = new ArrayList<Integer>();
		arr_r_x = new ArrayList<Double[]>();
		arr_r_y = new ArrayList<Double[]>();
		
		ctx = Freenect.createContext();
		if (ctx.numDevices() > 0) {
			kinect = ctx.openDevice(0);
		} else {
			System.err.println("WARNING: No kinects detected");
		}
		window = new JFrame("calibrate camera");
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		rgbVideo = new KinectRGBVideo(kinect);
		rgbVideo.addMouseListener(ml);
		window.setContentPane(rgbVideo);
	}
	
	MouseListener ml = new MouseAdapter() {
		public void mouseClicked(MouseEvent e) {
			Point p = e.getPoint();
			//pixel points of click
			int pX = p.x - C_X;
			int pY = C_Y - p.y;
			
			//real world points of click
			double rZ = .5;
			//double depth = getDepth(x, y);
			double rX = Double.parseDouble(JOptionPane.showInputDialog("enter real world x value"));
			double rY = Double.parseDouble(JOptionPane.showInputDialog("enter real world y value"));
			System.out.println("x: " + pX + ", y: " + pY);
			
			arr_p_x.add(pX);
			arr_p_y.add(pY);
			arr_r_x.add(new Double[] {rX, rY, rZ, 1d, 0d, 0d, 0d, 0d, -pX*rX, -pX*rY, -pX*rZ});
			arr_r_y.add(new Double[] {0d, 0d, 0d, 0d, rX, rY, rZ, 1d, -pY*rX, -pY*rY, -pY*rZ});
			
		}
	};
	
	private void createMatrix() {
//		ArrayList<Integer> r_array = 
	}
	
	public static void main(String [] args) {
		CalibrateCam cc = new CalibrateCam();
	}
	
	
	//q = (r^tr)^-1r^tp
	


}
