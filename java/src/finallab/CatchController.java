package finallab;

import java.lang.Math;
import java.util.*;
import java.io.*;
import java.awt.Point;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import april.util.*;
import april.jmat.*;

import finallab.lcmtypes.*;

public class CatchController implements LCMSubscriber
{
	Projectile predictor;
	BallDetector viewer;
	boolean display = true;
	boolean logs = false;
	boolean started = false;

	final static long TURNINGSCALE = (long)((.2)*1000000000.0);
	final static long MOVEMENTSCALE = (long)((1.0)*1000000000.0);
	final static double BOX_Y = .1;
	final static double BOT_DIST_FROM_KINECT_X = -1.505;
	final static double BOT_DIST_FROM_KINECT_Y = 0.91;
	final static double TARGET_DIST_FROM_KINECT_X = -1.505;
	final static double TARGET_DIST_FROM_KINECT_Y = 1.67;
	final static double TARGET_WIDTH = 1.22;
	final static double TARGET_HEIGHT = .91;
	final static double TARGET_MIN_Y = TARGET_DIST_FROM_KINECT_Y - TARGET_HEIGHT/2;
	final static double TARGET_MAX_Y = TARGET_DIST_FROM_KINECT_Y + TARGET_HEIGHT/2;
	final static double TARGET_MIN_X = TARGET_DIST_FROM_KINECT_X - TARGET_WIDTH/2;
	final static double TARGET_MAX_X = TARGET_DIST_FROM_KINECT_X + TARGET_WIDTH/2;
	final static double TARGET_PADDING = 0.3;
	final static double BOT_THETA = Math.PI/2;//Math.atan2(BOT_DIST_FROM_KINECT_Y,BOT_DIST_FROM_KINECT_X);
	final static int BALLS_TO_WAIT_ON = 8;
	final static double HUMAN_LOC_Y = TARGET_DIST_FROM_KINECT_Y;
	final static double HUMAN_LOC_X = 2.0;
	LCM  lcm;
	
	Object ballLock;

	// x=-2.07
	// y = 2.11
	CatchController(boolean _display, boolean _logs)
	{
		
		display = _display;
		logs = _logs;
		predictor = new Projectile(_display);
		ballLock = new Object();
		if(!logs)
		{
			viewer = new BallDetector(true);
			viewer.start();
		}
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}
		catch(IOException e){
			lcm = LCM.getSingleton();
		}
		

		//recieve = LCM.getSingleton();
		//receive = lcm;
		lcm.subscribe("6_BALL", this);	
		lcm.subscribe("6_RESET", this);
		lcm.subscribe("6_SCORE_HUMAN", this);
		lcm.subscribe("6_SCORE_ROBOT", this);
		lcm.subscribe("6_SCORE_RESET", this);	
		if (display) {
			lcm.subscribe("6_WAYPOINT", this);
			lcm.subscribe("6_POSE", this);
			lcm.subscribe("6_BATTERY", this);
		}
	}

	public Point3D convertToPointRobotNonMat(double[] point)
	{
		double xDist = -BOT_DIST_FROM_KINECT_X + point[0];
		double yDist = -BOT_DIST_FROM_KINECT_Y + point[1];
		return new Point3D(xDist,yDist,0);
	}
	
	public Point3D convertToPointRobot(double[] point)
	{
		double [][] robo_point_array = {{point[0]}, {point[2]}, {0}, {1}};
		double [][] trans_mat = {{1,0,0,BOT_DIST_FROM_KINECT_X},{0,1,0,BOT_DIST_FROM_KINECT_Y},{0,0,1,0},{0,0,0,1}};
		Matrix global_point_mat = new Matrix(trans_mat).times(new Matrix(LinAlg.rotateY(BOT_THETA))).times(new Matrix(robo_point_array));
		return new Point3D(global_point_mat.get(0, 0), global_point_mat.get(1, 0), 0);
	}

	Point3D determineBounceCatch(ArrayList<Parabola> bounces) {
		if ((bounces == null) || (bounces.size() == 0)) {
			return null;
		}

		Parabola bestParab = null;
		double bestX = 999999d;
		int best = -1;
		for (int i = 0; i < bounces.size(); i++) {
			if (bounces.get(i) == null) {
				continue;
			}
			Parabola curr = bounces.get(i);
			double [] land = curr.pred_landing;
			if (land[0] > TARGET_MAX_X || land[0] < (TARGET_MIN_X) || 
				land[1] > (TARGET_MAX_Y + TARGET_PADDING) || land[1] < (TARGET_MIN_Y)) {
//				System.out.println("bounce ind " + i + " oob");
				continue;
			}
//			double dist = Math.hypot(curr.pred_landing[0] - TARGET_DIST_FROM_KINECT_X, curr.pred_landing[1] - TARGET_DIST_FROM_KINECT_Y); 
			if (land[0] < bestX) {
				bestParab = curr;
				best = i;
				bestX = land[0];
//				System.out.println("best dist: " + bestDist);
			}
	//		System.out.println("Points x y z :" + point[0] + " " + point[1] + " "
	//				+ point[2]);
	
		}
		
		if (bestParab == null) {
			System.out.println("no balls in target zone");
			return null;
		}
//		System.out.println("best ind: " + best);
		// convert from points of kinect to points in front of robot
		Point3D landing = convertToPointRobotNonMat(bestParab.pred_botlanding);
		//if there's a bounce, wait for 5 balls before we start sending waypoints again
		if (bestParab.balls_in_parab == 0 || bestParab.balls_in_parab > (BALLS_TO_WAIT_ON - 1))
			return landing;
		else {
			System.out.println("waiting for better estimate");
			return null;
		}

	}

	public void catchStateMachine()
	{
		ArrayList<Parabola> bounces;
		//start up video stuff
		//obtain projectile stuff
		//go to catchmode
		int state = 0;
		int nextState = 0;
		ball_t ball;
		Point3D newWayPoint = new Point3D(0.0,0.0,0.0);
		System.out.println("waiting for landing point");
		do {
			
			
			bounces = predictor.getParabolas();
		}
		while(bounces == null || bounces.size() == 0 || !bounces.get(0).valid);
		
		System.out.println("done waiting for bounces");
		double[][] startingBounces = new double[2][bounces.size()];
		for(int i = 0; i < bounces.size(); i++)
		{
			startingBounces[0][i] = bounces.get(i).pred_landing[0];
			startingBounces[1][i] = bounces.get(i).pred_landing[1];
		}
		while(true)
		{
			
			// System.out.println("x:"+ball.x+ " y:" +ball.y+ " z:" +ball.z);
			bounces = predictor.getParabolas();
			Point3D land;
			
			switch(state)
			{
				//waiting state
				//waiting for signal of endpoint from projectile
				//once recieved points determine weather good to grab on first bounce second bounce etc...
				case 0:
					// determine if able to catch on first bounce or second bounce
					land = determineBounceCatch(bounces);
					if(land == null)
					{
				
						nextState = 0;
						break;
					}
					if(land.x != newWayPoint.x || land.y != newWayPoint.y || land.z != newWayPoint.z)
					{
						//bot takes waypoints looking down positive x
						xyt_t spot = new xyt_t();
						spot.utime = TimeUtil.utime();
						spot.xyt[0] = land.y;
						spot.xyt[1] = -land.x;
						spot.xyt[2] = 0d;
						spot.goFast = true;
						newWayPoint = land.clone();
						// go to point at bounce index
						if (!logs) {
							lcm.publish("6_WAYPOINT",spot);
							System.out.println("sending waypoint - LX: " + spot.xyt[0] + ", LY: " + spot.xyt[1] + "  (" + System.currentTimeMillis() + ")");
						}
						
//						for(int i = 0; i < bounces.size(); i++)
//						{
//							double[] endPoint = bounces.get(i).pred_landing;
//							double xDiff = startingBounces[0][i] -endPoint[0], yDiff = startingBounces[1][i] - endPoint[i];
//							System.out.println("Difference From Starting in Bounce " + i + " x: " +xDiff+ " y: " +yDiff);
															
//						}
					}
					else {
						System.out.println("waypoints are the same");
					}
					//to continuously send waypoints of updated position of bounce index 0
					 nextState = 0;
					//to send the waypoint once when first calculated
//					nextState = 1;
				break;
				
				//retrieve state
				//go catch at point	
				case 1:
					//keep updating point with better trajectory
					//check to see if panda bot has reached position
					//if(pose distance is < distance to travel)
						//goToWayPoint(bounces.get(bounceIndex));
				break;

				//deliver state
				case 2:
					//go to point of human interaction 
					//goToWayPoint(humanPoint)
				break;
				
				//deliver_wait
				case 3: 
					//wait for human to take out ball
				break;
				
				//return
				case 4:
					//go to point of origin
				break;
				default:
					//error state
				break;

			}
			state = nextState;
//			System.out.println("waiting for ball");
			synchronized(ballLock) {
				try {
					ballLock.wait();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	public void blockGameStateMachine()
	{
		int state = 0;
		while(true)
		{
			switch(state)
			{
				//waiting state
				case 0:
				break;
				//retrieve state
				//go catch at point
				case 1:
				break;
				//determine if blocked
				case 2:
				break;
				//go back to previous point
				case 3:
				break;
				default:
				break;

			}
		}
	}


	public static void main(String[] args)
	{
		
		boolean logs = false;
		boolean display = true;
		for(int i = 0; i < args.length; i++)
		{
			if(args[i].equals("log"))
				logs = true;
			if(args[i].equals("speed"))
				display = false;
			if(args[i].equals("DEMO"))
			{
				@SuppressWarnings("unused")
				CatchController demo = new CatchController(true, true);
				while(true);
			}
		}
		CatchController cc = new CatchController(display, logs);
		cc.catchStateMachine();
	}

	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) {
		if (channel.equals("6_BALL")) {
//			System.out.println("got ball from detector (" + System.currentTimeMillis() + ")");
			ball_t ball = null;
			try {
				ball = new ball_t(dins);
//				ball.y += KINECT_HEIGHT;
			} catch (IOException e) {
				e.printStackTrace();
			}
//			if(!started)
//			{
//				started = true;
//				xyt_t spot = new xyt_t();
//				spot.utime = TimeUtil.utime();
//				spot.xyt[0] = 0.5d;
//				spot.xyt[1] = 0.0d;
//				spot.xyt[2] = 0.0d;
//				spot.goFast = true;
//				// go forward to save time
////				System.out.println("sending go straight waypoint");
//				lcm.publish("6_WAYPOINT",spot);
//			}
			predictor.update(ball);
			synchronized (ballLock) {
				ballLock.notify();
			}
		}
		else if (channel.equals("6_RESET")) {
			if (started) {
				predictor.reset();
				// go home
				started = false;
			}
		}
		else if(channel.equals("6_POSE"))
		{
			try
			{
				bot_status_t curr_bot_status = new bot_status_t(dins);
				double _y = curr_bot_status.xyt[1];
				curr_bot_status.xyt[1] = curr_bot_status.xyt[0] + BOT_DIST_FROM_KINECT_Y;
				curr_bot_status.xyt[0] = -_y + BOT_DIST_FROM_KINECT_X;  
				predictor.drawRobot(curr_bot_status);
			}
			catch(Exception e)
			{
				System.out.println("Dins coding error 6_POSE");
			}
		}
		else if(channel.equals("6_SCORE_HUMAN"))
		{
			xyt_t home = new xyt_t();
			home.utime = TimeUtil.utime();
			home.xyt[0] = 0.0d;
			home.xyt[1] = 0.0d;
			home.xyt[2] = 0.0d;
			
			if (!logs) {
				lcm.publish("6_WAYPOINT",home);
				System.out.println("logs is false");
			}
			else {
				predictor.scoreBoard.addToHuman();
				System.out.println("logs is true");
			}

		}
		else if(channel.equals("6_SCORE_ROBOT"))
		{
			xyt_t human = new xyt_t();
			human.utime = TimeUtil.utime();
			human.xyt[0] = 0.0d;
			human.xyt[1] = 0.0d;
			human.xyt[2] = 0.0d;
			if (!logs)
				lcm.publish("6_WAYPOINT",human);
			else
				predictor.scoreBoard.addToRobot();
		}
		else if(channel.equals("6_SCORE_RESET"))
		{
			xyt_t home = new xyt_t();
			home.utime = TimeUtil.utime();
			home.xyt[0] = 0.0d;
			home.xyt[1] = 0.0d;
			home.xyt[2] = 0.0d;
			if (!logs)
				lcm.publish("6_WAYPOINT",home);
			else
				predictor.scoreBoard.clearScoreboard();	
		}

		if (display)
		{
			if (channel.equals("6_WAYPOINT")) {
				try
				{
					xyt_t point = new xyt_t(dins);
					predictor.drawRobotEnd(new Point3D(BOT_DIST_FROM_KINECT_X, BOT_DIST_FROM_KINECT_Y, 0), point.xyt);
				}
				catch(Exception e)
				{
					System.out.println("Dins coding error 6_WAYPOINT");
				}
			}
			else if (channel.equals("6_BATTERY")) {
				try
				{
					battery_t battery = new battery_t(dins);
					predictor.PrintBattery(battery.voltage);
				}
				catch(Exception e)
				{
					System.out.println("Dins coding error 6_BATTERY");
				}
			}
		}

		
		
	}
		
}

	