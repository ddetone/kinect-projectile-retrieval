package finallab;

import java.util.*;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.WindowListener;
import java.awt.event.WindowEvent;
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

	int points;

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
		window.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		window.setVisible(true);
		rgbVideo = new KinectRGBVideo(kinect);
		rgbVideo.addMouseListener(ml);
		window.setSize(640, 480);
		window.setContentPane(rgbVideo);
		window.addWindowListener(new WindowClose(this));


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
	
	public class WindowClose implements WindowListener {
		CalibrateCam cc;
		public WindowClose(CalibrateCam _cc) {
			cc = _cc;
		}
		public void windowClosing(WindowEvent e) {
			cc.arr_p_x.addAll(arr_p_y);
			cc.arr_r_x.addAll(arr_r_y);

			double [][] p_arr = new double[cc.arr_p_x.size()][1];
			double [][] r_arr = new double[cc.arr_r_x.size()][11];

			for (int i = 0; i < cc.arr_p_x.size(); i++) {
				p_arr[i][0] = (double)cc.arr_p_x.get(i);
				for (int j = 0; j < 11; j++) {
					r_arr[i][j] = (double)cc.arr_r_x.get(i)[j];
				}
			}
			cc.p = new Matrix(p_arr);
			cc.r = new Matrix(r_arr);
			Matrix q = (cc.r.transpose().times(cc.r)).inverse().times(cc.r.transpose()).times(cc.p);
			System.out.println(q.toString());
		}
		public void windowOpened(WindowEvent e)        {   }
      	public void windowClosed(WindowEvent e)         {   }
	    public void windowActivated(WindowEvent e)     {   }
	    public void windowDeactivated(WindowEvent e) {   }
	    public void windowIconified(WindowEvent e)      {   }
	    public void windowDeiconified(WindowEvent e)   {   }
	}

	// private void createMatrix() {
	// 	arr_p_x.addAll(arr_p_y);
	// 	arr_r_x.addAll(arr_r_y);

	// 	double [][] p_arr = new double[arr_p_x.size()][1];
	// 	double [][] r_arr = new double[arr_r_x.size()][11];

	// 	for (int i = 0; i < arr_p_x.size(); i++) {
	// 		p_arr[i][0] = (double)arr_p_x.get(i);
	// 		for (int j = 0; j < 11; j++) {
	// 			r_arr[i][j] = (double)arr_r_x.get(i)[j];
	// 		}
	// 	}
	// 	p = new Matrix(p_arr);
	// 	r = new Matrix(r_arr);
	// 	Matrix q = (r.transpose().times(r)).inverse().times(r.transpose()).times(p);
	// 	System.out.println(q.toString());
	// }
	
	public static void main(String [] args) {
		CalibrateCam cc = new CalibrateCam();
		// while(true) {
		// 	try
		// }
	}
	
	
	//q = (r^tr)^-1r^tp
	


}
