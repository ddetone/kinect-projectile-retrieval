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
	final boolean verbose2 = true;
	LCM lcm;
	bot_status_t bot_status;


	//Robot is actively following if isFollow = true else if does not move
	static boolean isFollow = false;
	static double[] cXYT = new double[3];
	static double[] dXYT = new double[3];
	static double angleToDest;
	static double errorDist, errorAngle;
	static double left, right;

	static final double MAX_SPEED = 1.0f;
	static final double SET_SPEED = 0.9f;
	static final double ALLOWED_ANGLE = 360;
	static final double STRAIGHT_DIST = 0.5; 
	static final double DEST_DIST = 0.20; 

	//double Kp_turn = 0.7;
	//double Kp = 1;
	//double Kd_turn = 0.001;
	//double Kd = 0.001;

	//int state = 0;
	//boolean turnEnd = false;
	//The PID controller for finer turning
	double[] KPID = new double[]{0.46, 0.006, 0.10};
	//double[] KPID = new double[]{0.46, 0.015, 0.0};

	PidController pidAngle = new PidController(KPID[0], KPID[1], KPID[2]);


	PathFollower()
	{
		this.lcm =  LCM.getSingleton();
		pidAngle.setIntegratorClamp(10);
		
		errorAngle = 0;
		left = SET_SPEED;
		right = SET_SPEED;

		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
		lcm.subscribe("6_PARAMS", this);
	}

	void calcErrors()
	{
		errorDist = LinAlg.distance(new double[]{cXYT[0],cXYT[1]}, new double[]{dXYT[0], dXYT[1]});
		
		angleToDest = Math.atan2(dXYT[1]-cXYT[1],dXYT[0]-cXYT[0]);
		double curAngle = cXYT[2];
		while(curAngle > Math.PI)curAngle -= 2 * Math.PI;
		while(curAngle < -Math.PI)curAngle += 2 * Math.PI;		

		errorAngle = angleToDest-curAngle;



		if(verbose)System.out.println("curAngle:" + Math.toDegrees(curAngle) +
				"angleToDest:" + Math.toDegrees(angleToDest)
				+ "  errorAngle:" + Math.toDegrees(errorAngle));
		
		if(verbose)System.out.printf("errorDist:%f\n",errorDist);

	}


	void moveRobot()
	{
		
		double pid = pidAngle.getOutput(errorAngle);
		//if(verbose) System.out.println("PID preclamp: "+pid);
		//pid = clampPid(pid);


		if(verbose)System.out.println("pid:" + pid);
				//+ "  integrator: " + pidAngle.integral);
		

		right = SET_SPEED + pid;
		left = SET_SPEED - pid;		

		if(verbose)System.out.println("left:" + left
				+ "  right: " + right);

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

		if(verbose)System.out.println();

		lcm.publish("6_DIFF_DRIVE", motor);
	}
	
	//to stop the robot just in case
	void stop()
	{
		setMotorCommand(0.0F, 0.0F);
		isFollow = false;
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
						if(verbose)System.out.println("reached waypoint");						
						stop();
					}
					else if (errorDist < STRAIGHT_DIST)
					{
						if(verbose)System.out.println("continue straight");					
						setMotorCommand(SET_SPEED,SET_SPEED);
					}
					else	
					{
						if(verbose)System.out.println("drive to waypoint");
						moveRobot();

					}
				}
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				xyt_t dest = new xyt_t(dins);
				dXYT[0] = dest.xyt[0];
				dXYT[1] = dest.xyt[1];
			
				if (errorAngle > Math.abs(Math.toRadians(ALLOWED_ANGLE)))
				{
					if(verbose)System.out.printf("angle to waypoint too big, greater than: %f\n", ALLOWED_ANGLE);
					if(verbose)System.out.printf("calculated angle to waypoint is: %f\n", errorAngle);
					isFollow = false;
					stop();
				}
				else
				{
					isFollow = true;
				}
			}
			else if (channel.equals("6_PARAMS")) {
				xyt_t params = new xyt_t(dins);
				pidAngle.changeParams(params.xyt);
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
