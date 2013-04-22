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

public class PoseGenerator extends Thread implements LCMSubscriber
{
	//This class would publish pose messages periodically

	LCM lcm;
	motor_feedback_t motors;

	public int[] encoder_curr;
	public int[] encoder_prev;

	//pose global
	//public static double poseG[] = new double[3];
	public bot_status_t bot;
	public battery_t battery;

	//Determined emperically
	static final double metersPerTick = 0.000194;
	static final double base = 18.8/100.0;

	boolean first;

	Pimu pimu;
	volatile boolean running;
	

	PoseGenerator() 
	{
		encoder_curr = new int[2];
		encoder_prev = new int[2];
		bot = new bot_status_t();
		battery = new battery_t();
		first = true;
		running = true;
		pimu = new Pimu(false);
		pimu.calibrate();

		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}catch(IOException e){
			this.lcm = LCM.getSingleton();
		}
		lcm.subscribe("6_MOTOR_FEEDBACK", this);
//		lcm.subscribe("6_BATTERY", this);
		lcm.subscribe("6_RESET", this);
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MOTOR_FEEDBACK"))
			{
				motors = new motor_feedback_t(dins);
				generatePose();

//				System.out.println("utime:" + bot_status.utime);
//				System.out.println("X:" + bot_status.xyt[0]);
//				System.out.println("Y:" + bot_status.xyt[1]);
//				System.out.println("T:" + bot_status.xyt[2]);
//				System.out.println();

			}
			else if(channel.equals("6_BATTERY")){
				battery = new battery_t(dins);
				lcm.publish("6_BATTERY", battery);
			}
			else if (channel.equals("6_RESET")) {
				bot.xyt[0] = 0;
				bot.xyt[1] = 0;
				bot.xyt[2] = 0;
				pimu.recalibrate();
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public void generatePose()
	{
		//get current encoder values
		encoder_curr[0] = motors.encoders[0];
		encoder_curr[1] = motors.encoders[1];

		if(first)
		{
			encoder_prev[0] = encoder_curr[0];
			encoder_prev[1] = encoder_curr[1];
			first = false;
		}

		//calculated difference in encoders
		double encoder_delta[] = new double[]{(encoder_curr[0] - encoder_prev[0]), (encoder_curr[1] - encoder_prev[1])};

		double[] xyt_T = new double[3];
		xyt_T[0] = (encoder_delta[1] + encoder_delta[0]) / 2 * metersPerTick;
		xyt_T[1] = 0;
		xyt_T[2] = (encoder_delta[1] - encoder_delta[0]) / base * metersPerTick;

		encoder_prev[0] = encoder_curr[0];
		encoder_prev[1] = encoder_curr[1];

		//previous global pose
		double[] xyt_A = new double[3];
		xyt_A[0] = bot.xyt[0];
		xyt_A[1] = bot.xyt[1];
		xyt_A[2] = bot.xyt[2];

		double ca = Math.cos(xyt_A[2]);
		double sa = Math.sin(xyt_A[2]);

		double[] xyt_B = new double[3];
		xyt_B[0] = xyt_T[0] * ca - xyt_T[1] * sa + xyt_A[0];
		xyt_B[1] = xyt_T[0] * sa + xyt_T[1] * ca + xyt_A[1];
		xyt_B[2] = xyt_A[2] + xyt_T[2];

		//computes covarience matrix using the A,B,T matrices
//		computeCov(xyt_A, xyt_B, xyt_T);

		//update the LCM data type (bot) with new pose
		bot.xyt[0] = xyt_B[0];
		bot.xyt[1] = xyt_B[1];
		//bot.xyt[2] = xyt_B[2];
		bot.xyt[2] = pimu.yaw;
		bot.yaw = pimu.yaw;
	
		/*
		yawsum += bot.yaw;
		System.out.printf("yaw sum: %f\n", yawsum);
		*/
	
		//get PIMU data (XYZ and RPY)
		double[] XYZ = pimu.getXYZdot();
		double[] RPY = pimu.getRPYdot();
		bot.xyt_dot[0] = XYZ[0]; 
		bot.xyt_dot[1] = XYZ[1];
		bot.xyt_dot[2] = RPY[2]; //2 is yaw

		bot.utime = TimeUtil.utime();
//		bot.cov = sigmaB;

		bot.voltage = battery.voltage;
		lcm.publish("6_POSE",bot);
		
		/*
		try{
			Thread.sleep(33);
		}catch(InterruptedException e)
		{}
		*/

	}
	

	
	public static void main(String [] args) {
		PoseGenerator pose = new PoseGenerator();
		while (true) {
			try {
				Thread.sleep(1000);
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		
	}
}
