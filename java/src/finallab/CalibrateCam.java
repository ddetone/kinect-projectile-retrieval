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
	JFrame windowD;
	KinectRGBVideo rgbFeed;
	KinectDepthVideo depthFeed;
	
	final static int C_X = KinectRGBVideo.WIDTH / 2;
	final static int C_Y = KinectRGBVideo.HEIGHT / 2;
	final static double CAMERA_HEIGHT = 0.052;
	
	ArrayList<Integer> arr_p_x;
	ArrayList<Integer> arr_p_y;
	ArrayList<Double[]> arr_r_x;
	ArrayList<Double[]> arr_r_y;
	
	Matrix r;
	Matrix p;

	int points;

	final boolean rgb;

	ParameterGUI pg;

	int clicks = 0;
	Point p1;

	CalibrateCam(boolean _rgb)
	{
		rgb = _rgb;
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
		KinectVideo cameraFeed;
		if (rgb) {
			rgbFeed = new KinectRGBVideo(kinect, null, true);
			// depthFeed = new KinectDepthVideo(kinect, true);
			// depthFeed.addMouseListener(depthPrinter);
			// windowD = new JFrame("depth");
			// windowD.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
			// windowD.setVisible(true);
			// windowD.setContentPane(depthFeed);
			// windowD.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);

			cameraFeed = rgbFeed;
		}
		else {
			depthFeed = new KinectDepthVideo(kinect, null, true);
			cameraFeed = depthFeed;
		}
		// cameraFeed.addMouseListener(calibrater);
		cameraFeed.addMouseListener(xyPrinter);
		window.setContentPane(cameraFeed);
		// window.add(pg);
		window.setSize(KinectVideo.WIDTH, KinectVideo.HEIGHT);
		window.addWindowListener(new WindowClose(this));

		// pg = new ParameterGUI();
		// pg.addDoubleSlider("f", "f", 200, 1000, 200);
		// JFrame slider = new JFrame("calibrate f");
		// slider.add(pg);
		// slider.setSize(80, 20);
		// slider.setVisible(true);
	}
	
	MouseListener calibrater = new MouseAdapter() {
		public void mouseClicked(MouseEvent e) {

			Point p = e.getPoint();

			//pixel points of click
			int pX = p.x - C_X;
			int pY = C_Y - p.y;

			
			// real world points of click
			// double rZ = .85;
			double rZ = 0d;
			if (rgb)
				rZ = Double.parseDouble(JOptionPane.showInputDialog("enter real world depth"));
			else
				rZ = (double)depthFeed.getDepthFromDepthPixel(p);
			double rX = Double.parseDouble(JOptionPane.showInputDialog("enter real world x value"));
			double rY = Double.parseDouble(JOptionPane.showInputDialog("enter real world y value"));
			System.out.println("x: " + pX + ", y: " + pY + ", dep: " + rZ);
			
			arr_p_x.add(pX);
			arr_p_y.add(pY);
			arr_r_x.add(new Double[] {rX / rZ, rZ, 0d});
			arr_r_y.add(new Double[] {rY / rZ, 0d, rZ});
			
		}
	};
	MouseListener depthPrinter = new MouseAdapter() {
		public void mouseClicked(MouseEvent e) {
			System.out.println((double)depthFeed.getDepthFromDepthPixel(e.getPoint()));			
		}
	};
	MouseListener xyPrinter = new MouseAdapter() {
		public void mouseClicked(MouseEvent e) {
			Point p = e.getPoint();
			//pixel points of click
			int pX = p.x - C_X;
			int pY = C_Y - p.y;
						if (clicks == 0) {
				p1 = p;
				clicks++;
			}
			else {
				// double f = pg.gd("f");
				// double x1 = (p1.x) * (1 / f);
				// double x2 = (p.x) * (1 / f);
				System.out.println("x diff: " + (p.x - p1.x));
				clicks = 0;
			}
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
			double [][] r_arr = new double[cc.arr_r_x.size()][3];

			for (int i = 0; i < cc.arr_p_x.size(); i++) {
				p_arr[i][0] = (double)cc.arr_p_x.get(i);
				for (int j = 0; j < 3; j++) {
					r_arr[i][j] = (double)cc.arr_r_x.get(i)[j];
				}
			}
			cc.p = new Matrix(p_arr);
			cc.r = new Matrix(r_arr);
			// System.out.println(cc.r.toString());
			Matrix q = ((((cc.r.transpose()).times(cc.r)).inverse()).times(cc.r.transpose())).times(cc.p);
			System.out.println(q.toString());
			System.out.println("f: " + q.get(0, 0));
			System.out.println("cx: " + q.get(1, 0));
			System.out.println("cy: " + q.get(2, 0));
			// System.out.println(q.toString());
		}
		public void windowOpened(WindowEvent e){   }
      	public void windowClosed(WindowEvent e){   }
	    public void windowActivated(WindowEvent e){   }
	    public void windowDeactivated(WindowEvent e){   }
	    public void windowIconified(WindowEvent e){   }
	    public void windowDeiconified(WindowEvent e){   }
	}
	
	public static void main(String [] args) {
		if (args.length != 1) {
			System.out.println("No type specified...\n\nOptions:\n-r | RGB camera calibration\n-d | Depth camera calibration");
			return;
		}
		boolean rgbCal;
		if (args[0].equals("-r")) {
			rgbCal = true;
		}
		else {
			rgbCal = false;
		}
		CalibrateCam cc = new CalibrateCam(rgbCal);
		// while(true) {
		// 	try
		// }
	}
	
	
	//q = (r^tr)^-1r^tp
	


}
