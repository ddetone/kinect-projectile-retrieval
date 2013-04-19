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

	final boolean verbose = false;
	final boolean verbose2 = false;
	static boolean time = false;
	LCM lcm;
	bot_status_t bot_status;

	ParameterGUI pg;
	JFrame jf;


	
	//Robot is actively following if isFollow = true else if does not move
	/*
	static boolean isFollow = false;
	static boolean stop = false;
	static boolean calc_turn_params = false;
	static boolean drive_straight = false;
	static boolean dest_rotate = false;
	static boolean waypoint = false;
	*/

	static double[] cXYT = new double[3];
	volatile static double[] dXYT = new double[3];
	volatile static boolean dFast = true;
	volatile static boolean newWaypoint = false;

	static double angleToDest;
	static double errorDist, errorAngle;
	static double prev_errorDist;



	//static double left, right;
	//static double tvolts;
	//static double straightAngle;

	static final double DEFAULT_VOLTAGEOFFSET = 0.15;
	static double voltageOffset = DEFAULT_VOLTAGEOFFSET;

	static final double MAX_SPEED = 1.0f;
	static final double FAST_SPEED = 0.95f;
	static final double MED_SPEED = 0.3f;
	static final double SLOW_SPEED = 0.3f;
	static final double MAX_TURNSPEED = 0.6;
	static final double FAST_STRAIGHT_ANGLE = Math.toRadians(13);
	static final double SLOW_STRAIGHT_ANGLE = Math.toRadians(1);

	static final double MEDDEST_DIST = 0.08;
	static final double SLOWDEST_DIST = 0.04; 
	static final double LEAVE_DIST_BUFFER = 0.01;
	static final double PREVDIST_BUFFER = 0.005f;
	static final double SLOW_DOWN_DIST = 0.4f;
	static final double LEAVE_DIST = MEDDEST_DIST + LEAVE_DIST_BUFFER;

	static final double SK_PID = 0.36;
	static final double SI_PID = 0.0;
	static final double SD_PID = 28000.0;

	static final double TK_PID = 0.28;
	static final double TI_PID = 0.0;
	static final double TD_PID = 38000.0;	

	static final double HK_PID = 0.05;
	static final double HI_PID = 0.0;
	static final double HD_PID = 10000.0;	
	//double Kp_turn = 0.7;
	//double Kp = 1;
	//double Kd_turn = 0.001;
	//double Kd = 0.001;

	//int state = 0;
	//boolean turnEnd = false;
	//The PID controller for finer turning
	
	double[] sPID = new double[]{SK_PID, SI_PID, SD_PID}; //PID for straight driving
	double[] tPID = new double[]{TK_PID, TI_PID, TD_PID};	 //PID for turning
	double[] hPID = new double[]{HK_PID, HI_PID, HD_PID};	//PID for rotating home
	

	PidController sPIDAngle = new PidController(sPID[0], sPID[1], sPID[2]);
	PidController tPIDAngle = new PidController(tPID[0], tPID[1], tPID[2]);
	PidController hPIDAngle = new PidController(hPID[0], hPID[1], hPID[2]);

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


		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
		lcm.subscribe("6_PARAMS", this);

	}

	void calcErrors()
	{
		errorDist = LinAlg.distance(new double[]{cXYT[0],cXYT[1]}, new double[]{dXYT[0], dXYT[1]});
		
		// if (dest_rotate)
		// 	angleToDest = dXYT[2];
		// else
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
		double angle = dXYT[2] - cXYT[2];
		while(angle > Math.PI)angle -= 2 * Math.PI;
		while(angle < -Math.PI)angle += 2 * Math.PI;	
		return angle;
	}


	void moveRobotStraight(double angle, double speed)
	{
		double pid = sPIDAngle.getOutput(angle);

		if(verbose)System.out.println("sPID:" + pid);
				//+ "  integrator: " + pidAngle.integral);

		double right = speed + pid;
		double left = speed - pid;	

		setMotorCommand(left, right);
	}

	void turnRobot(double angle, boolean fast)
	{

		if(verbose)System.out.printf("Turning...\n");

		double pid;
		if (fast)
			pid = tPIDAngle.getOutput(angle);
		else
			pid = hPIDAngle.getOutput(angle);


		if (pid > 0)
			pid += voltageOffset;
		else
			pid -= voltageOffset;

		pid = LinAlg.clamp(pid,-MAX_TURNSPEED, MAX_TURNSPEED);

		double right = pid;
		double left = -pid;
		
		if(verbose)System.out.printf("pid:%f\n",pid);

		setMotorCommand(left, right);
	

	}
/*
	boolean checkHalt()
	{
		if (dFast)
		{
			if (errorDist < FASTDEST_DIST)
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
		}
		else
		{
			if (errorDist < SLOWDEST_DIST)
			{
				if(verbose)System.out.printf("SLOWLY drove to dest, now slow rotate\n");
				return true;
			}
			else if ((prev_errorDist+PREVDIST_BUFFER) < errorDist)
			{
				if(verbose)System.out.printf("STOP...prev_errorDist < errorDist\n");
				if(verbose)System.out.printf("PrevDist:%f , Dist:%f",prev_errorDist, errorDist);
				return true;
			}
		}

		return false;
	}
*/
/*
	boolean checkStraight()
	{
		double straightAngle;
		if (dFast || !dest_rotate) //if not gofast, ust faststop angle
			straightAngle = Math.toRadians(pg.gd("faststopangle"));
		else 
			straightAngle = Math.toRadians(pg.gd("slowstopangle"));

		if (errorAngle >= 0)
		{
			if (errorAngle < straightAngle)
			{
				if(verbose)System.out.printf("Drive straight boolean is true\n");
				return true;
			}
		}
		else if (errorAngle < 0)
		{
			if (errorAngle > (-1)*straightAngle)
			{
				if(verbose)System.out.printf("Drive straight boolean is true\n");
				return true;
			}
		}
		return false;
	}
*/
	void stopBot()
	{
		setMotorCommand(0.0F, 0.0F);
		//isFollow = false;
		prev_errorDist = 9999;
		//stop = false;
		return;
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

				/*
				if (isFollow) //if a waypoint is recieved
				{
					calcErrors();

					if (drive_straight && checkHalt())
					{
						drive_straight = false;
						if (dFast)
							stop = true;
						else
						{
							dest_rotate = true;
							setMotorCommand(0.0F, 0.0F);
						}
					}

					if (!dFast && dest_rotate && checkStraight()) //2nd rotate phase, slow only
					{
						dest_rotate = false;
						stop = true;
					}
						
					if (!drive_straight && checkStraight() && !dest_rotate) //1st rotate phase, both fast and slow
							drive_straight = true;
					
					if (drive_straight)
					{
						if (errorDist < SLOW_DIST || !dFast)
						{
							if(verbose)System.out.printf("Drive slow homie\n");					
							moveRobotStraight(SLOW_SPEED);
						}
						else	
						{
							if(verbose)System.out.printf("Drive fast\n");
							moveRobotStraight(FAST_SPEED);
						}
					}
					else if (!dest_rotate)
						turnRobot();


					if(verbose)System.out.println("errorAngle:" +
						errorAngle + " errorDist:" + errorDist);
		
					prev_errorDist = errorDist;
					if (time)System.out.printf("%d\n",System.currentTimeMillis());

					if (stop)
						stopBot();					
				}
				*/

			}
			else if(channel.equals("6_WAYPOINT"))
			{
				System.out.printf("\nWaypoint RECIEVED\n");
				if (time)System.out.printf("%d\n",System.currentTimeMillis());

				xyt_t dest = new xyt_t(dins);

				dXYT[0] = dest.xyt[0];
				dXYT[1] = dest.xyt[1];
				dXYT[2] = dest.xyt[2];
				dFast = dest.goFast;

				newWaypoint = true;


				//drive_straight = false;
				//dest_rotate = false;
				//isFollow = true;
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
		PathStateMachine psm = new PathStateMachine(pl);

		while(true)
		{
			psm.stateMachine();
		}
	}
}
