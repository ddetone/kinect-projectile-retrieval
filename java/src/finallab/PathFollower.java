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


public class PathFollower implements LCMSubscriber
{

	public enum State {
		STOP, ROTATE_FAST, ROTATE_SLOW, ROTATE_TO_ANGLE, GO_FAST, GO_MED, GO_SLOW
	}

	final boolean verbose = false;
	final boolean verbose2 = false;
	static boolean time = false;
	int printcount = 0;
	

	LCM lcm;
	bot_status_t bot_status;
	Object poseMonitor;
	Object waypointMonitor;
	ParameterGUI pg;
	JFrame jf;
	//PathStateMachine psm;

	State state = State.STOP;
	State nextState = State.STOP;
	State prevState = State.STOP;

	volatile double[] cXYT = new double[3];
	volatile double[] dXYT = new double[3];
	volatile boolean dFast = true;
	//boolean newWaypoint = false;
	//boolean anyWaypoint = false;

	volatile double angleToDest;
	volatile double errorDist, errorAngle;
	volatile double prev_errorDist;


	static final double DEFAULT_VOLTAGEOFFSET = 0.15;
	static double voltageOffset = DEFAULT_VOLTAGEOFFSET;

	static final double MAX_SPEED = 1.0f;
	static final double FAST_SPEED = 1.0f;
	static final double MED_SPEED = 0.3f;
	static final double SLOW_SPEED = 0.3f;
	static final double MAX_TURNSPEED = 0.6;
	static final double FAST_STRAIGHT_ANGLE = Math.toRadians(40);
	static final double SLOW_STRAIGHT_ANGLE = Math.toRadians(2);

	static final double MEDDEST_DIST = 0.25;
	static final double SLOWDEST_DIST = 0.04; 
	static final double LEAVE_DIST_BUFFER = 0.01;
	static final double PREVDIST_BUFFER = 0.005f;
	static final double SLOW_DOWN_DIST = 0.15f;
	static final double LEAVE_DIST = MEDDEST_DIST + LEAVE_DIST_BUFFER;

	static final double SK_PID = 0.36;
	static final double SI_PID = 0.0;
	static final double SD_PID = 28000.0;

	static final double TK_PID = 0.20;
	static final double TI_PID = 0.0;
	static final double TD_PID = 38000.0;	

	static final double HK_PID = 0.05;
	static final double HI_PID = 0.0;
	static final double HD_PID = 30000.0;	

	
	double[] sPID = new double[]{SK_PID, SI_PID, SD_PID}; //PID for straight driving
	double[] tPID = new double[]{TK_PID, TI_PID, TD_PID};	 //PID for turning fast
	double[] hPID = new double[]{HK_PID, HI_PID, HD_PID};	//PID for turning slow

	PidController sPIDAngle = new PidController(sPID[0], sPID[1], sPID[2]);
	PidController tPIDAngle = new PidController(tPID[0], tPID[1], tPID[2]);
	PidController hPIDAngle = new PidController(hPID[0], hPID[1], hPID[2]);

	PathFollower(boolean gs)
	{
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}
		catch(IOException e){
			lcm = LCM.getSingleton();
		}
		//sPIDAngle.setIntegratorClamp(10);
		
		errorAngle = 0;
		prev_errorDist = 9999;

		if (gs == true)
		{
			pg = new ParameterGUI();
			//pg.addDoubleSlider("turnRate", "Turn Rate", 0d, 1d, 0.573d);
			//pg.addDoubleSlider("straightAngleRate","Straight Angle Rate", 0d, 2d, DEF_STRAIGHT_ANGLE);
			pg.addDoubleSlider("voltageOffset", "Voltage Offset", 0d, 0.5d, 0.15d);
			//pg.addDoubleSlider("faststopangle", "faststopangle", 0d, 90d, Math.toDegrees(FAST_STRAIGHT_ANGLE));
			//pg.addDoubleSlider("slowstopangle", "slowstopangle", 0d, 90d, Math.toDegrees(SLOW_STRAIGHT_ANGLE));

			pg.addDoubleSlider("skp", "skp", 0d, 1d, SK_PID);
			pg.addDoubleSlider("ski", "ski", 0d, 1d, SI_PID);
			pg.addDoubleSlider("skd", "skd", 0d, 80000d, SD_PID);

			pg.addDoubleSlider("tkp", "tkp", 0d, 1d, TK_PID);
			pg.addDoubleSlider("tki", "tki", 0d, 1d, TI_PID);
			pg.addDoubleSlider("tkd", "tkd", 0d, 80000d, TD_PID);

			pg.addDoubleSlider("hkp", "hkp", 0d, 1d, HK_PID);
			pg.addDoubleSlider("hki", "hki", 0d, 1d, HI_PID);
			pg.addDoubleSlider("hkd", "hkd", 0d, 80000d, HD_PID);

			pg.addListener(new ParameterListener() {
				public void parameterChanged(ParameterGUI pg, String name)
				{
					
					if(name == "skp" || name == "ski" || name == "skd")
					{
						double[] params = new double[3];
						params[0] = pg.gd("skp");
						params[1] = pg.gd("ski");
						params[2] = pg.gd("skd");
						sPIDAngle.changeParams(params);
					}
						
						if(name == "tkp" || name == "tki" || name == "tkd")
						{
							double[] params = new double[3];
							params[0] = pg.gd("tkp");
							params[1] = pg.gd("tki");
							params[2] = pg.gd("tkd");
							tPIDAngle.changeParams(params);
						}
						
						if(name == "hkp" || name == "hki" || name == "hkd")
						{
							double[] params = new double[3];
							params[0] = pg.gd("hkp");
							params[1] = pg.gd("hki");
							params[2] = pg.gd("hkd");
							hPIDAngle.changeParams(params);
						}
						if (name == "voltageOffset")
							voltageOffset = pg.gd("voltageOffset");

					}
				});


			jf = new JFrame("PathFollow Params");
			jf.add(pg);
			jf.setSize(600,350);
			jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			jf.setVisible(true);
		}

		poseMonitor = new Object();
		waypointMonitor = new Object();

		//psm = new PathStateMachine(this);

		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
		lcm.subscribe("6_PARAMS", this);

	}

	void calcErrors()
	{
		errorDist = LinAlg.distance(new double[]{cXYT[0],cXYT[1]}, new double[]{dXYT[0], dXYT[1]});

		angleToDest = Math.atan2(dXYT[1]-cXYT[1],dXYT[0]-cXYT[0]);
		
		double curAngle = cXYT[2];
		errorAngle = angleToDest-curAngle;

		while(errorAngle > Math.PI)errorAngle -= 2 * Math.PI;
		while(errorAngle < -Math.PI)errorAngle += 2 * Math.PI;
	}

	void printError()
	{
		if(verbose)System.out.printf("Error Angle: %f, Error Distance: %f\n", errorAngle, errorDist);		
	}

	double calcAngleToWaypointTheta()
	{
		double angle = cXYT[2] - dXYT[2];
		while(angle > Math.PI)angle -= 2 * Math.PI;
		while(angle < -Math.PI)angle += 2 * Math.PI;	
		return angle;
	}


	void moveRobotStraight(double speed)
	{
//		System.out.println("begin drive straight (" + System.currentTimeMillis() + ")");
		double pid = sPIDAngle.getOutput(errorAngle);
//		System.out.println("after pid calc (" + System.currentTimeMillis() + ")");
		// if(verbose)printError();
//		if(verbose2)System.out.println("sPID:" + pid);
				//+ "  integrator: " + pidAngle.integral);

		double right = speed + pid;
		double left = speed - pid;	
		System.out.println("right: " + right + ", left: " + left);

//		if(verbose)printError();
		

		setMotorCommand(left, right);
	}

	void turnRobot(boolean toAngle)
	{


		double pid;
		if (!toAngle)
		{
			if (dFast)
				pid = tPIDAngle.getOutput(errorAngle);
			else
				pid = hPIDAngle.getOutput(errorAngle);
		}
		else
			pid = hPIDAngle.getOutput(calcAngleToWaypointTheta());


		if (pid > 0)
			pid += voltageOffset;
		else
			pid -= voltageOffset;

		pid = LinAlg.clamp(pid,-MAX_TURNSPEED, MAX_TURNSPEED);

		double right = pid;
		double left = -pid;
		
		if(verbose)System.out.printf("Turning...\n");
//		if(verbose)printError();
		
		if(verbose2)System.out.printf("pid:%f\n",pid);

		setMotorCommand(left, right);
	

	}

	void stopBot()
	{
		setMotorCommand(0.0F, 0.0F);
		//isFollow = false;
		prev_errorDist = 9999;
		//stop = false;
		return;
	}

	void setMotorCommand(double left, double right)
	{
//		System.out.println("before motorcommand (" + System.currentTimeMillis() + ")");
		diff_drive_t motor = new diff_drive_t();

		motor.left_enabled = false;
		motor.right_enabled = false;

		motor.utime = TimeUtil.utime();

		left = LinAlg.clamp(left, -MAX_SPEED, MAX_SPEED);
		right = LinAlg.clamp(right, -MAX_SPEED, MAX_SPEED);

		motor.left = (float)left;
		motor.right = (float)right;

		//if(verbose)System.out.println();
//		System.out.println("publishing motors L: " + left + ", R: " + right + "(" + System.currentTimeMillis() + ")");
		lcm.publish("6_DIFF_DRIVE", motor);
	}	

	public void stateMachine()
	{
		if(verbose)printState();
		// if (state != prevState)
		// 	if(verbose)printError();
		switch(state)
		{				

			case STOP:
				if(prevState != State.STOP)
					stopBot();

				//if (newWaypoint == true)
				//{
					//if(verbose)System.out.printf("New Waypoint in StateMachine\n");
					//newWaypoint = false;

					if (errorDist > LEAVE_DIST)
					{
						if(verbose)System.out.printf("ErrorDist > LEAVE_DIST\n");

						if (dFast == true)
						{
							if (Math.abs(errorAngle) < Math.abs(FAST_STRAIGHT_ANGLE))
							{
								nextState = State.GO_FAST;
//								System.out.println("calling drive straight (" + System.currentTimeMillis() + ")");
								moveRobotStraight(FAST_SPEED);
							}
							else
								nextState = State.ROTATE_FAST;
						}
						else
							nextState = State.ROTATE_SLOW;
					}
				//}
				break;

			case ROTATE_SLOW:
				
				if (Math.abs(errorAngle) < Math.abs(SLOW_STRAIGHT_ANGLE))
				{
					nextState = State.GO_SLOW;
					break;
				}
				turnRobot(false);
				break;

			case ROTATE_FAST:

				if (Math.abs(errorAngle) < Math.abs(FAST_STRAIGHT_ANGLE))
				{
					nextState = State.GO_FAST;
					break;
				}
				turnRobot(false);
				break;

			case ROTATE_TO_ANGLE:

				double rotate_angle = calcAngleToWaypointTheta();
				if(verbose)System.out.printf("Rotate Angle Error:%f\n", rotate_angle);
				if (Math.abs(rotate_angle) < Math.abs(SLOW_STRAIGHT_ANGLE))
				{
					nextState = State.STOP;
					break;
				}
				turnRobot(true);
				
				/*try {
					Thread.sleep(100);
				}
				catch(Exception e) {}*/

				break;

			case GO_SLOW:

				if (errorDist < SLOWDEST_DIST)
				{
					if(verbose)System.out.printf("Current Waypoint Dist < Slow Speed Destination Dist\n");
					nextState = State.ROTATE_TO_ANGLE;
					break;
				}
				moveRobotStraight(SLOW_SPEED);

//			case GO_MED:
//				
//				//if we're driving away from the waypoint, stop
////				if (errorDist > (prev_errorDist+PREVDIST_BUFFER))
////				{
////					if(verbose)System.out.printf("Current Waypoint Dist > Prev Dist to Waypoint + BufVal\n");
////					if(verbose2)System.out.printf("PrevDist:%f , Dist:%f",prev_errorDist, errorDist);
////					nextState = State.STOP;
////					break;
////				}
//				if (errorDist > SLOW_DOWN_DIST)
//				{
//					if(verbose)System.out.printf("Speeding up\n");
//					nextState = State.GO_FAST;
//					break;
//				}
//				if (errorDist < MEDDEST_DIST)
//				{
//					if(verbose)System.out.printf("Current Waypoint Dist < Slow Speed Destination Dist\n");
//					nextState = State.STOP;
//					break;
//				}
//				moveRobotStraight(MED_SPEED);
//				break;

			case GO_FAST:

				//if we're driving away from the waypoint, stop
//				if (errorDist > (prev_errorDist+PREVDIST_BUFFER))
//				{
//					if(verbose)System.out.printf("Current Waypoint Dist > Prev Dist to Waypoint + BufVal\n");
//					nextState = State.STOP;
//					break;
//				}
				if (errorDist < MEDDEST_DIST)
				{
					if(verbose)System.out.printf("Reached FASTDEST_DIST\n");
					nextState = State.STOP;
					break;
				}
				/*
				if (errorDist < SLOW_DOWN_DIST)
				{
					if(verbose)System.out.printf("Slowing Down\n");
					nextState = State.GO_MED;
					break;							
				}*/
				moveRobotStraight(FAST_SPEED);
				break;
		}

		prev_errorDist = errorDist;
		prevState = state;
		state = nextState;

		synchronized(poseMonitor) {
			try {
				poseMonitor.wait();
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
//		System.out.println("wake up (" + System.currentTimeMillis() + ")");
		/*
		try {
			Thread.sleep(10);
		}
		catch(Exception e) {}
		*/

	}

	public void printState()
	{
		
		if (state != prevState)
		{

			if (prevState != state) System.out.printf("\n");
			String statestring;
			if (state == State.STOP)
				statestring = "STOP";
			else if (state == State.ROTATE_FAST)
				statestring = "ROTATE_FAST";
			else if (state == State.ROTATE_SLOW)
				statestring = "ROTATE_SLOW";
			else if (state == State.ROTATE_TO_ANGLE)
				statestring = "ROTATE_TO_ANGLE";
			else if (state == State.GO_FAST)
				statestring = "GO_FAST";
			else if (state == State.GO_MED)
				statestring = "GO_MED";			
			else if (state == State.GO_SLOW)
				statestring = "DRIVE SLOW HOMIE";
			else
				statestring = "UNKNOWN";
			System.out.println("Current State is: " + statestring);
//			System.out.printf("Error Angle: %f, PreviousError Distance: %f, Error Distance: %f\n", errorAngle, prev_errorDist, errorDist);
//			System.out.printf("Current position, X:%f, Y:%f, T:%f\n",cXYT[0],cXYT[1],cXYT[2]);
		}
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_WAYPOINT"))
			{
//				System.out.printf("Waypoint RECIEVED\n");

				xyt_t dest = new xyt_t(dins);
//				System.out.println("moving to waypoint (" + dest.xyt[0] + ", "  + dest.xyt[1] + ") (" + System.currentTimeMillis() + ")");

				dXYT[0] = dest.xyt[0];
				dXYT[1] = dest.xyt[1];
				dXYT[2] = dest.xyt[2];
				dFast = dest.goFast;
				//dFast = true;
				//newWaypoint = true;
				calcErrors();
				
				synchronized(poseMonitor) {
					poseMonitor.notify();
				}

//				System.out.println("done waypoint received (" + System.currentTimeMillis() + ")");
//				System.out.printf("notified\n");

				
			}
			else if(channel.equals("6_POSE"))
			{
				bot_status = new bot_status_t(dins);
				cXYT[0] = bot_status.xyt[0];
				cXYT[1] = bot_status.xyt[1];
				cXYT[2] = bot_status.xyt[2];

				calcErrors();

				synchronized(poseMonitor) {
					poseMonitor.notify();
				}
					

			}


			else if (channel.equals("6_PARAMS")) {
				xyt_t params = new xyt_t(dins);
				sPIDAngle.changeParams(params.xyt);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception
	{
		
		boolean gs = false;
		for(int i = 0; i < args.length; i++)
		{
			if(args[i].equals("gs"))
				gs = true;
		}
		

		//boolean gs = true;
		PathFollower pl = new PathFollower(gs);

		/*
		synchronized(pl.waypointMonitor) {
			try {
				pl.waypointMonitor.wait();
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}

		System.out.printf("passed wait\n");*/
		while(true)
		{
			pl.stateMachine();
		}

		// Object lcmMonitor = new Object();
		// synchronized(lcmMonitor) {
		// 		try {
		// 			lcmMonitor.wait();
		// 		}
		// 		catch(Exception e) {
		// 			e.printStackTrace();
		// 		}
		// 	}
	}
}
