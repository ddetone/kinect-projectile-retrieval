import java.awt.*;
import java.io.*;
import java.lang.Math.*;
import java.util.*;

import javax.swing.*;

import lcm.lcm.*;
import april.jcam.*;
import april.jmat.*;
import april.jmat.geom.*;
import java.awt.event.*;
import april.util.*;
import april.vis.*;
import botlab.lcmtypes.*;

public class MovementControl
{
	//speed lookup tables for both wheels
	double leftSpeedLU[] = new double[24];
	double rightSpeedLU[] = new double[24];
	volatile int leftCount = 0;
	volatile int rightCount = 0;

	int pLeftCount = 0;
	int pRightCount = 0;

	static final double B = .183; 
	
	volatile double left = 0;
	volatile double right = 0;
	
	// volatile double degrees = 0;

	volatile double pLeft = 0.3;
	volatile double pRight = 0.3;
	
	volatile double kp = 0.6d;
	volatile double kd = 4d;

	double kp_theta = 0.003;
	
	double kp_target = 0.0001;
//	double kd_target = 0.001;

	final double metersPerTick = 0.000193;
	final double metersPerTickRotate = 0.000086;
	// final double metersPerTick = 0.0002;
	double b_m = 0.082;
	double b_ticks = b_m / metersPerTick;

	double b_ticks_rotate = b_m / metersPerTickRotate;

	static LCM lcm_pc;

	volatile boolean pause;
	

	GyroMonitor yaw;
	MovementControl(GyroMonitor gm)
	{


		double step  = .02;
		leftSpeedLU[12] = .33;
		rightSpeedLU[12] = .3;
		rightSpeedLU[11] = 0;
		leftSpeedLU[11] = 0;
		rightSpeedLU[10] = -.3;
		leftSpeedLU[10] = -.33;
		for(int i = 1; i < 11; i++)
		{
			leftSpeedLU[12+i] = leftSpeedLU[12+(i-1)] + step;
			rightSpeedLU[12+i] = rightSpeedLU[12+(i-1)] + step;
			leftSpeedLU[10-i] = leftSpeedLU[10-(i-1)] - step;
			rightSpeedLU[10-i] = rightSpeedLU[10-(i-1)] - step;
		}

		try
		{
			lcm_pc = new LCM("udpm://239.255.76.8:7667?ttl=1");
		}
		catch(Exception E)
		{
			System.out.print("FUDGE!");
		}
		
		yaw = gm;
		
	}

	public boolean moveToPoint(float distance, double theta)
	{
		
			System.out.println(theta);
			rotateToTheta(theta);
			PID(distance);
			return true;
	}
	public double getAngleFromPoint(Point3D center) {
		double depth = center.z;
		double laserAdjust = .015;
		// System.out.println(center.x);
		double xP = center.x + laserAdjust;
		double theta = Math.atan2(depth,xP) - Math.PI/2;
		// System.out.println(theta+ " is theta");
		return theta;
	}

	public void centerToTriangle(Point3D triangleCenter)
	{
		//find depths of point
		double depth = triangleCenter.z;
		double laserAdjust = .015;
		System.out.println(triangleCenter.x);
		double xP = triangleCenter.x + laserAdjust;
		double theta = Math.atan2(depth,xP) - Math.PI/2;
		System.out.println(theta+ " is theta");
		rotateToTheta(theta);
	}

	public void publishMotor()
	{
		try{
				LCM lcm = LCM.getSingleton();
	
				diff_drive_t msg = new diff_drive_t();
	
				msg.utime = TimeUtil.utime();
	
				msg.left_enabled = false;
				msg.right_enabled = false;
	
				msg.left = (float)left;
				msg.right = (float)right;

				lcm.publish("DIFF_DRIVE",msg);
		}
		catch(Throwable t) {
			 System.out.println("Error: Exception thrown");
		}

	}
	
	public void rotateToTarget(TriangleBlob target, int width, int height, TriangleDetector td) {

		final int centerIm = width / 2;
		int centerT = target.center[0];
		
		int error = centerIm - centerT;
		
		double offsetR;
		double offsetL;
	
		offsetR = 0d;

		if (error > 0) {
			// offsetR = 0.15;
			offsetL = -0.2;
			// right = offsetR + .1;
			left = offsetL - .1;
		}
		else {
			// offsetR = -0.15;
			offsetL = 0.2;
			// right = offsetR - .1;
			left = offsetL + .1;
		}
		right = offsetR;

		publishMotor();
		try{
			Thread.sleep(200);
		}
		catch(Exception e){};
		TriangleBlob currTarget = target;
		
		while(//Math.abs(error) > 0
				Math.abs(error) > 10)
		{

			long time = System.currentTimeMillis();
			
			//this takes too long
			TriangleBlob newTarget;

			newTarget = td.getNewTriangle(currTarget, error);
//			System.out.println(System.currentTimeMillis() - time);
			
			int newError = centerIm - newTarget.center[0];

			error = newError;			
			//orient wheels in correct direction
			if (error > 0) {
				offsetR = 0.15;
				offsetL = -0.18;
			}
			else {
				offsetR = -0.15;
				offsetL = 0.18;
			}
			double appliedChange = kp_target*error;
			
			if(Math.abs(appliedChange) > .1)
			{
				appliedChange = (appliedChange / Math.abs(appliedChange)) * 0.1;
			}
			left = offsetL - appliedChange;
			right = offsetR + appliedChange;
			publishMotor();

			currTarget = newTarget;
		}
		
	}

	public boolean rotateToTheta(double theta) {
		return rotateDegrees(Math.toDegrees(theta));
	}

	//make robot face target
	public boolean rotateDegrees(double angle) {
		
		float motorR = .12f;
		float motorL = .12f;
		double offsetR;
		double offsetL;
	
		if (angle > 0) {
			offsetR = motorR;
			offsetL = -motorL;
		}
		else {
			offsetR = -motorR;
			offsetL = motorL;
		}

		MotorPublisher mp = new MotorPublisher(this);
		mp.start();

		//move forward to straighten out front wheel
		left = 0.2d;
		right = 0.2d;
		try {
			Thread.sleep(200);
		}
		catch(Exception e){}
		double currDegrees = yaw.getDegrees();
		final double destDegrees = currDegrees + angle;
		System.out.println("dest position: " + destDegrees);
		double error = 999999999;
		
		if (Math.abs(angle) > 20) {
			kp_theta = 0.003;
			while(/*Math.abs(error) > 0*/Math.abs(error) > 1)
			{
				try{
					Thread.sleep(20);
				}
				catch(Exception e){};
				
				currDegrees = yaw.getDegrees();
				
				error = destDegrees - currDegrees;

				
				if (error > 0) {
					offsetR = motorR;
					offsetL = -motorL;
				}
				else {
					offsetR = -motorR;
					offsetL = motorL;
				}
				double appliedChange = kp_theta*error;
				
				if(Math.abs(appliedChange) > .2)
				{
					appliedChange = (appliedChange / Math.abs(appliedChange)) * 0.2;
				}
				left = offsetL - appliedChange;
				right = offsetR + appliedChange;
	
	
			}
		}
		else {		
			kp_theta = 0.03;	

			left = motorL;
			
			right = motorR;
			
			// only use one wheel for more precision
			while (Math.abs(error) > 1) {
				// orient wheels in correct direction
				try {
					Thread.sleep(20);
				} 
				catch (Exception e) {};
				double appliedChange = kp_theta * error;
				
				if (Math.abs(appliedChange) > .2) {
					appliedChange = (appliedChange / Math.abs(appliedChange)) * 0.2;
				}
				if (angle > 0) {
					right = offsetR + appliedChange;
				}
				else {				
					left = offsetL + appliedChange;
				}
				
				
				currDegrees = yaw.getDegrees();
				
				error = destDegrees - currDegrees;
				
			}
		}
		
		
		left = 0;
		right = 0;
		publishMotor();

		currDegrees = yaw.getDegrees();
		// System.out.println("done degrees: " + currDegrees);
		System.out.println("off by " + (currDegrees - destDegrees) + " degrees");
		return true;
	}


	//makes robot go in a straight line
	public double PID(double distance)
	{
		CountUpdater cu = new CountUpdater(this);
		cu.start();
		try {
			Thread.sleep(500);
		}
		catch(Exception e){}

		double distanceTravelled = 0;
		double totalTheta = 0;
		double derError = 0;
		//decode current distance
		double offsetL = 1.0;
		double offsetR = 1.0 - .06;
		System.out.println("Traveling to Speed L:" + offsetL + " R:" +offsetR);

		left = offsetL;
		right = offsetR;

		int previousTicksL = leftCount;
		int previousTicksR = rightCount;
		//sleep to get the motors a chance to run
		publishMotor();
		//decode the travel distance and error from the encoders
		previousTicksL = leftCount;
		previousTicksR = rightCount;
		//get 200ms worth of change in ticks
		try {
			Thread.sleep(200);
		}
		catch(Exception e){}

		int changeInTicksLeft  = Math.abs(leftCount - previousTicksL);
		int changeInTicksRight = Math.abs(rightCount- previousTicksR);
		// System.out.println("changeInTicksRight: " + changeInTicksRight);
		// System.out.println("changeInTicksLeft: " + changeInTicksLeft);
		//calculate the error

		int dist_ticks = (changeInTicksRight + changeInTicksLeft) / 2;
		double dist_m = Math.abs(dist_ticks * metersPerTick);
		double theta = (changeInTicksRight - changeInTicksLeft) / b_ticks;
		totalTheta = theta;
		double error = Math.sin(theta) * dist_m;
		distanceTravelled += Math.abs(Math.cos(theta) * dist_m);
		// if (error == 0) {error = 21;}
		// System.out.println("initial error: " + error);

		pause = false;
		// System.out.println("distanceTravelled: " + distanceTravelled);
		while(distanceTravelled < distance)
		{
			// System.out.printf("error: %f totalTheta: %f\n", error, totalTheta);
			// System.out.println("kp: " + kp + ", kd: " + kd);
			// if (theta > 0) {
				double appliedChange = kp*error + kd*derError;
				if(Math.abs(appliedChange) > .3)
				{
					if(appliedChange < 0)
						appliedChange = -.3;
					else
						appliedChange = .3;
				}
				//changed
				left = offsetL - appliedChange;
				right = offsetR + appliedChange;
			// System.out.println("left speed: " + left + ", right speed: " + right);

			//update motor signals to align correctly
			previousTicksL = leftCount;
			previousTicksR = rightCount;
			try{
				Thread.sleep(200);
			}
			catch(Exception e){};
			changeInTicksLeft  = Math.abs(leftCount - previousTicksL);
			changeInTicksRight = Math.abs(rightCount- previousTicksR);
			//System.out.printf("ctL: %d ctR: %d\n", changeInTicksLeft,changeInTicksRight);

			dist_ticks = (changeInTicksRight + changeInTicksLeft) / 2;
			dist_m = Math.abs(dist_ticks * metersPerTick);
			theta = (changeInTicksRight - changeInTicksLeft) / b_ticks;
			totalTheta += theta;
			derError = Math.sin(totalTheta) * dist_m; 
			error += derError;
			distanceTravelled += Math.abs(Math.cos(totalTheta) * dist_m);
		}
		left = 0;
		right = 0;
		System.out.println("done pid");
		return distanceTravelled;
	}


	private class MotorPublisher extends Thread {
		MovementControl p;
		MotorPublisher(MovementControl _p) {
			p = _p;
		}
		public void run() {
			while(true) {
				try{
					Thread.sleep(10);
				}
				catch(Exception e){};
				p.publishMotor();
			}
		}
	}
	public class CountUpdater extends Thread implements LCMSubscriber {

		MovementControl p;
		int bullshit = 0;

		public CountUpdater(MovementControl _p) {
			p = _p;
			
		}

		public void run() {
			LCM.getSingleton().subscribe("MOTOR_FEEDBACK", this);
			lcm_pc.subscribe("8_PAUSE", this);
			lcm_pc.subscribe("8_RESUME", this);
			lcm_pc.subscribe("8_PID", this);
			

			while(true) {
				try{
						Thread.sleep(10);
					}
				catch(Exception e){};
				p.publishMotor();
			}
		}

		@Override
		public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) {
			if(channel.equals("MOTOR_FEEDBACK"))
			{
				motor_feedback_t motorStats = null;
				try {
					publishMotor();
					motorStats = new motor_feedback_t(dins);
					p.leftCount = motorStats.encoders[1];
					p.rightCount = motorStats.encoders[0];
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				
			}
			else if (channel.equals("8_PAUSE")) {

				// p.pLeft = p.left;
				// p.pRight = p.right;
				// System.out.println("left ticks: " + (p.leftCount - p.pLeftCount) + "   right ticks: " + (p.rightCount - p.pRightCount));
				// p.pLeftCount = p.leftCount;
				// p.pRightCount = p.rightCount;
				p.pause = true;
				System.out.println("pausing");
				// p.left = 0;
				// p.right = 0;
			}
			else if (channel.equals("8_RESUME")) {
				// p.pause = false;
				p.PID(2.0, 8);
				// p.left = p.pLeft;
				// p.right = p.pRight;
			}
			else if (channel.equals("8_PID")) {
				try {
					pid_params msg = new pid_params(dins);
					p.kp = msg.kp;
					p.kp_theta = msg.kp;
					p.kd = msg.kd;
				}
				catch(Exception e) {
					e.printStackTrace();
				}
			}

		}
	}

}
