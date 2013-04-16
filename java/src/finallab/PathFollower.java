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

	final boolean verbose = true;
	final boolean verbose2 = false;
	LCM lcm;
	bot_status_t bot_status;


	//Robot is actively following if isFollow = true else if does not move
	static boolean isFollow = false;
	static boolean stop = false;
	static double[] cXYT = new double[3];
	static double[] dXYT = new double[3];
	static double angleToDest;
	static double errorDist, errorAngle;
	static double prev_errorDist;
	static double left, right;

	static final double MAX_SPEED = 1.0f;
	static final double FAST_SPEED = 0.9f;
	static final double SLOW_SPEED = 0.4f;
	static final double STRAIGHT_ANGLE = Math.toRadians(20);
	static final double SLOW_DIST = 0.35; 
	static final double DEST_DIST = 0.08; 

	//double Kp_turn = 0.7;
	//double Kp = 1;
	//double Kd_turn = 0.001;
	//double Kd = 0.001;

	//int state = 0;
	//boolean turnEnd = false;
	//The PID controller for finer turning
	
	double[] sPID = new double[]{0.46, 0.006, 0.10}; //PID for straight driving
	double[] tPID = new double[]{0.35, 0.0, -30000};	 //PID for turning
	static final double TURN_OFFSET = 0.2;

	PidController sPIDAngle = new PidController(sPID[0], sPID[1], sPID[2]);
	PidController tPIDAngle = new PidController(tPID[0], tPID[1], tPID[2]);

	PathFollower()
	{
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}
		catch(IOException e){
			lcm = LCM.getSingleton();
		}
		sPIDAngle.setIntegratorClamp(10);
		
		errorAngle = 0;
		prev_errorDist = 9999;

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


	void moveRobotStraight(double speed)
	{
		double pid = sPIDAngle.getOutput(errorAngle);
		//if(verbose) System.out.println("PID preclamp: "+pid);
		//pid = clampPid(pid);

		if(verbose)System.out.println("sPID:" + pid);
				//+ "  integrator: " + pidAngle.integral);

		double right = speed + pid;
		double left = speed - pid;	

		setMotorCommand(left, right);
	}

	void turnRobot()
	{
		double pid = tPIDAngle.getOutput(errorAngle);
		//double pid = 0.8;
		
		//+ "  integrator: " + pidAngle.integral);
		if (pid > 0)
			pid += TURN_OFFSET;
		else
			pid -= TURN_OFFSET;

		double right = pid;
		double left = -pid;
		if(verbose)System.out.println("tPID:" + pid);	

		setMotorCommand(left, right);


	}

	double clampPid(double pid)
	{
		if(pid > 0)
			pid += 0.1;
		else
			pid -= 0.1;

		return(pid);
	}

	void setMotorCommand(double left, double right)
	{
		diff_drive_t motor = new diff_drive_t();

		motor.left_enabled = false;
		motor.right_enabled = false;

		motor.utime = TimeUtil.utime();

		left = LinAlg.clamp(left, -MAX_SPEED, MAX_SPEED);
		right = LinAlg.clamp(right, -MAX_SPEED, MAX_SPEED);

		motor.left = (float)left;
		motor.right = (float)right;

		//if(verbose)System.out.println();

		lcm.publish("6_DIFF_DRIVE", motor);
	}
	
	void stop()
	{
		setMotorCommand(0.0F, 0.0F);
		isFollow = false;
		prev_errorDist = 9999;
		stop = false;
	}

	public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				bot_status = new bot_status_t(dins);
				cXYT[0] = bot_status.xyt[0];
				cXYT[1] = bot_status.xyt[1];
				cXYT[2] = bot_status.xyt[2];

				if (isFollow) //if a waypoint is recieved
				{
					calcErrors();
					
					if (errorDist < DEST_DIST)
					{
						isFollow = false;
						if(verbose)System.out.println("STOP...reached waypoint\n");						
						stop = true;
					}
					else if ((prev_errorDist+0.005) < errorDist)
					{
						isFollow = false;
						if(verbose)System.out.println("STOP...prev_errorDist < errorDist\n");
						if(verbose)System.out.printf("PrevDist:%f , Dist:%f",prev_errorDist, errorDist);						
						stop = true;	
					}
					else if (Math.abs(errorAngle) > Math.abs(STRAIGHT_ANGLE))
					{
						if(verbose)System.out.printf("Turning...\n");
						turnRobot();
					}
					else
						stop = true;

						/*
					else if (errorDist < SLOW_DIST)
					{
						if(verbose)System.out.println("Continue straight\n");					
						moveRobotStraight(SLOW_SPEED);
					}
					else	
					{
						if(verbose)System.out.println("Drive to waypoint\n");
						moveRobotStraight(FAST_SPEED);
					}*/

					if(verbose)System.out.println("errorAngle:" +
						Math.toDegrees(errorAngle) + " errorDist:" + errorDist);
		
					prev_errorDist = errorDist;

					if (stop)
						stop();
					
				}
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				System.out.printf("Waypoint RECIEVED\n");
				xyt_t dest = new xyt_t(dins);
				dXYT[0] = dest.xyt[0];
				dXYT[1] = dest.xyt[1];
				isFollow = true;
			}
			else if (channel.equals("6_PARAMS")) {
				xyt_t params = new xyt_t(dins);
				tPIDAngle.changeParams(params.xyt);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception
	{
		PathFollower pl = new PathFollower();

		//System.out.println(Math.atan2(12.0,0.0));
		while(true)
		{
			Thread.sleep(1000);
		}


	}


}
