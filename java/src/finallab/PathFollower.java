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
	static boolean time = true;
	LCM lcm;
	bot_status_t bot_status;

	ParameterGUI pg;
	JFrame jf;


	//Robot is actively following if isFollow = true else if does not move
	static boolean isFollow = false;
	static boolean stop = false;
	static boolean home_rotate = false;
	static boolean calc_straight = false;
	static boolean drive_straight = false;
	static double[] cXYT = new double[3];
	static double[] dXYT = new double[3];
	static double angleToDest;
	static double errorDist, errorAngle;
	static double prev_errorDist;
	static double left, right;
	static double tvolts;
	static double straightAngle;

	static final double MAX_SPEED = 1.0f;
	static final double MAX_TURNSPEED = 0.6f;
	static final double FAST_SPEED = 0.9f;
	static final double SLOW_SPEED = 0.4f;
	static final double PREVDIST_BUFFER = 0.005f;

	//static final double TURN_OFFSET = 0.2;
	static final double DEF_STRAIGHT_ANGLE = Math.toRadians(25);
	static final double SLOW_DIST = 0.35; 
	static final double DEST_DIST = 0.08; 

	//double Kp_turn = 0.7;
	//double Kp = 1;
	//double Kd_turn = 0.001;
	//double Kd = 0.001;

	//int state = 0;
	//boolean turnEnd = false;
	//The PID controller for finer turning
	
	double[] sPID = new double[]{0.46, 0.006, -20000}; //PID for straight driving
	double[] tPID = new double[]{0.30, 0.0, -30000};	 //PID for turning
	

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

		pg = new ParameterGUI();
		pg.addDoubleSlider("turnRate", "Turn Rate", 0d, 1d, 0.573d);
		pg.addDoubleSlider("straightAngleRate","Straight Angle Rate", 0d, 2d, 1.018d);
		pg.addDoubleSlider("voltageOffset", "Voltage Offset", 0d, 0.5d, 0.2d);
		jf = new JFrame("PathFollow Params");
		jf.add(pg);
		jf.setSize(400,100);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setVisible(true);


		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
		lcm.subscribe("6_PARAMS", this);

		tvolts = 0;
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
		if(verbose)System.out.printf("Turning...\n");

		//double pid = tPIDAngle.getOutput(errorAngle);

		if(verbose2)System.out.println("preclamp,tvolts:" + tvolts);

		double right = tvolts;
		double left = -tvolts;
		if(verbose)System.out.println("tvolts:" + tvolts);	

		setMotorCommand(left, right);

	}

	void calcTvolts()
	{

		tvolts = errorAngle*pg.gd("turnRate");
		double voltageOffset = pg.gd("voltageOffset");
		
		if(verbose2)System.out.printf("preoffset, tvolts: %f\n", tvolts);

		if (tvolts > 0)
			tvolts += voltageOffset;
		else
			tvolts -= voltageOffset;

		tvolts = LinAlg.clamp(tvolts,-MAX_TURNSPEED, MAX_TURNSPEED);

	}

	boolean checkHalt()
	{
		if (errorDist < DEST_DIST)
		{
			if(verbose)System.out.printf("STOP...reached waypoint\n");
			return true;
		}
		else if ((prev_errorDist+PREVDIST_BUFFER) < errorDist)
		{
			if(verbose)System.out.printf("STOP...prev_errorDist < errorDist\n");
			if(verbose)System.out.printf("PrevDist:%f , Dist:%f",prev_errorDist, errorDist);
			return true;
		}
		return false;
	}

	void calcStraightAngle()
	{
		if (tvolts == 0)
			straightAngle = Math.toRadians(20);
		else
			straightAngle = tvolts*pg.gd("straightAngleRate");

		straightAngle = Math.abs(straightAngle);

		if(verbose)System.out.printf("straightAngle:%f\n",straightAngle);
	}

	boolean checkStraight()
	{
		if (Math.abs(errorAngle) < straightAngle)
			return true;
		else
			return false;
	}
	
	void stop()
	{
		/*
		if (home_rotate)
		{
			System.out.printf("Rotate Home\n");
			errorAngle = -1*cXYT[2];
			home_rotate = false; 
		}
		else
		{*/
			setMotorCommand(0.0F, 0.0F);
			isFollow = false;
			prev_errorDist = 9999;
			stop = false;
		//}
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
					calcTvolts();

					if (!drive_straight && calc_straight) //only calculate this once per waypoint sent
						calcStraightAngle();

					if (checkHalt())
					{
						isFollow = false;
						stop = true;
						drive_straight = false;
					}

					if (checkStraight() || drive_straight)
					{
						isFollow = false;
						stop = true;
						/*
						if (errorDist < SLOW_DIST)
						{
							if(verbose)System.out.printf("Drive slow homie\n");					
							moveRobotStraight(SLOW_SPEED);
						}
						else	
						{
							if(verbose)System.out.printf("Drive fast\n");
							moveRobotStraight(FAST_SPEED);
						}*/
					}
					else
						turnRobot();


					if(verbose)System.out.println("errorAngle:" +
						Math.toDegrees(errorAngle) + " errorDist:" + errorDist);
		
					prev_errorDist = errorDist;
					if (time)System.out.printf("%d\n",System.currentTimeMillis());

					if (stop)
						stop();

					//isFollow = false;
					
				}
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				System.out.printf("\nWaypoint RECIEVED\n");
				if (time)System.out.printf("%d\n",System.currentTimeMillis());

				xyt_t dest = new xyt_t(dins);

				dXYT[0] = dest.xyt[0];
				dXYT[1] = dest.xyt[1];
				if (dest.xyt[2] == -1)
					home_rotate = true;

				// dXYT[0] = dest.xyt[1];
				// dXYT[1] = -dest.xyt[0];
				isFollow = true;
				calc_straight = true;
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
