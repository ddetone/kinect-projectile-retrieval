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
	KinectView viewer;
	boolean display = true;

	final long TURNINGSCALE = (long)((.2)*1000000000.0);
	final long MOVEMENTSCALE = (long)((1.0)*1000000000.0);
	final double BOT_DIST_FROM_KINECT_X = 0.0;
	final double BOT_DIST_FROM_KINECT_Y = 1.0;
	final double BOT_THETA = Math.PI/2;//Math.atan2(BOT_DIST_FROM_KINECT_Y,BOT_DIST_FROM_KINECT_X);
	LCM  lcm;
	LCM recieve;

	CatchController(boolean _display, boolean logs)
	{
		predictor = new Projectile();
		display = _display;
		if(!logs)
		{
			viewer = new KinectView(_display);
			viewer.start();
		}
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}
		catch(IOException e){
			lcm = LCM.getSingleton();
		}
		recieve = LCM.getSingleton();
		recieve.subscribe("6_BALL", this);	
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

	Point3D determineBounceCatch(ArrayList<Parabola> bounces)
	{
		if((bounces == null) || (bounces.size() == 0))
			return null;
		int i = 0;
		for(Parabola bounce: bounces)
		{
			double [] point = bounce.pred_landing;
			System.out.println("Ponits x y z :" + point[0] + " " + point[1] + " " + point[2]);
			long timeToMove = 0;
			//convert from points of kinect to points in front of robot
			Point3D landing = convertToPointRobotNonMat(point);
			return landing;
			/*double angle = Math.atan2(landing.y,landing.x);
			//if rotation is required then add to time .2 is scale factor for turning .2 sec per radian
			if(Math.abs(angle) > Math.PI/6)
				timeToMove = (long)(angle*TURNINGSCALE);
			double distance = Math.sqrt((landing.y*landing.y)+(landing.x*landing.x));
			//add time to timeToMove based on distance traveled moves 1 second per meter
			timeToMove += (long)(MOVEMENTSCALE*distance);
			long destinationTime = System.nanoTime();
			destinationTime += (long)(timeToMove)*1000000000;
			if(destinationTime < (long)(point[3]))
				return i; 
			i++;*/
		}
		return null;
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
		do {
			bounces = predictor.getParabolas();
		}
		while(!bounces.get(0).valid);
		
		while(true)
		{
			
			// System.out.println("x:"+ball.x+ " y:" +ball.y+ " z:" +ball.z);
			bounces = predictor.getParabolas();
			Point3D land;
			Point3D wayPoint = new Point3D(0.0,0.0,0.0);
			switch(state)
			{
				//waiting state
				//waiting for signal of endpoint from projectile
				//once recieved points determine weather good to grab on first bounce second bounce etc...
				case 0:
					// determine if able to catch on first bounce or second bounce
					land = determineBounceCatch(bounces);
					double r = Math.sqrt(land.x*land.x + land.y*land.y);
					double theta = Math.atan2(land.y, land.x);
					System.out.println("LX:" + land.x + "LY:" + land.y);
					xyt_t spot = new xyt_t();
					spot.xyt[0] = r;
					spot.xyt[2] = theta;
					if((land.x != wayPoint.x) || (land.y != wayPoint.y) || (land.z != wayPoint.z))
					{
					// go to point at bounce index
						lcm.publish("6_WAYPONT",spot);
						wayPoint = land;
						predictor.DrawBallsWithRobot(new Point3D(0.0,1.0,0.0),spot.xyt);
						return;
					}
					nextState = 1;
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
		for(int i = 0; i < args.length; i++)
		{
			if(args[i].equals("log"))
				logs = true;
		}
		CatchController cc = new CatchController(true, logs);
		cc.catchStateMachine();
	}

	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) {
		if (channel.equals("6_BALL")) {
			System.out.println("Got Ball");
			ball_t ball = null;
			try {
				ball = new ball_t(dins);
			} catch (IOException e) {
				e.printStackTrace();
			}
			predictor.update(ball);
		}
		
	}
		
}

	