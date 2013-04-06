package finallab;

import java.lang.Math;
import java.util.*;
import java.io.*;
import java.awt.Point;

import april.util.*;
import april.jmat.*;

import finallab.lcmtypes.*;

public class CatchController
{
	Projectile predictor;
	KinectView viewer;
	boolean display = false;

	final long TURNINGSCALE = (long)((.2)*1000000000.0);
	final long MOVEMENTSCALE = (long)((1.0)*1000000000.0);
	final double BOT_DIST_FROM_KINECT_X = 1.0;
	final double BOT_DIST_FROM_KINECT_Y = 3.0;
	final double BOT_THETA = Math.PI/2;//Math.atan2(BOT_DIST_FROM_KINECT_Y,BOT_DIST_FROM_KINECT_X);

	CatchController(boolean _display)
	{
		predictor = new Projectile();
		display = _display;
		viewer = new KinectView(_display);
	}

	public Point3D convertToPointRobot(double[] point)
	{
		Point3D answer = new Point3D();
		double [][] robo_point_array = {{point[0]}, {point[2]}, {0}, {1}};
		double [][] trans_mat = {{1,0,0,BOT_DIST_FROM_KINECT_X},{0,1,0,BOT_DIST_FROM_KINECT_Y},{0,0,1,0},{0,0,0,1}};
		Matrix global_point_mat = new Matrix(trans_mat).times(new Matrix(LinAlg.rotateZ(BOT_THETA))).times(new Matrix(robo_point_array));
		return new Point3D(global_point_mat.get(0, 0), global_point_mat.get(1, 0), 0);
	}

	int determineBounceCatch(ArrayList<double[]> bounces)
	{
		if((bounces == null) || (bounces.size() == 0))
			return -1;
		int i = 0;
		for(double[] point : bounces)
		{
			long timeToMove = 0;
			//convert from points of kinect to points in front of robot
			Point3D landing = convertToPointRobot(point);
			double angle = Math.atan2(landing.y,landing.x);
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
			i++;
		}
		return -1;
	}

	public void catchStateMachine()
	{
		ArrayList<double[]> bounces = new ArrayList<double[]>();
		//start up video stuff
		//obtain projectile stuff
		//go to catchmode
		int state = 0;
		int nextState = 0;
		ball_t ball;
		//bounces = predictor.update(ball);
		int bounceIndex = -1;
		while(true)
		{
			//update ball points
			//update projectile path
			
			ball = viewer.update();
			// System.out.println("x:"+ball.x+ " y:" +ball.y+ " z:" +ball.z);
			//bounces = predictor.update(ball);
			switch(state)
			{
				//waiting state
				//waiting for signal of endpoint from projectile
				//once recieved points determine weather good to grab on first bounce second bounce etc...
				case 0:
					//if gotten enough points to determine projectile begin analysis
						//determine if able to catch on first bounce or second bounce
						 //next state 1
					//otherwise go to state zero again
						//next state = 0
					if(bounces.size() == 0)
					{
						nextState = 0;
					}
					else
					{
						//determine if able to catch on first bounce or second bounce
						bounceIndex = determineBounceCatch(bounces);
						if(bounceIndex == -1)
						{
							nextState = 4;
						}
						else
						{
							//go to point at bounce index
							//goToWayPoint(bounces.get(bounceIndex));
						}
					}
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
		CatchController cc = new CatchController(true);
		cc.catchStateMachine();
	}
		
}

	