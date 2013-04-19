package finallab;

import java.io.*;
import java.util.*;

public class PathStateMachine
{
	public enum State {
		STOP, ROTATE_FAST, ROTATE_SLOW, ROTATE_TO_ANGLE, GO_FAST, GO_SLOW
	}

	volatile State state = State.STOP;
	State nextState = State.STOP;
	State prevState = State.STOP;
	PathFollower p = null;
	boolean verbose = true;
	boolean verbose2 = false;
	int printcount = 0;

	PathStateMachine(PathFollower _parent)
	{
		p = _parent;
	}

	public void stateMachine()
	{
			if(verbose)printState();
			switch(state)
			{				
				case STOP:
					if(prevState != State.STOP)
						p.stopBot();

					if (p.newWaypoint == true)
					{
						if(verbose)System.out.printf("New Waypoint in StateMachine\n");
						p.newWaypoint = false;
						p.calcErrors();

						if (p.errorDist > p.LEAVE_DIST)
						{
							if(verbose)System.out.printf("ErrorDist > LEAVE_DIST\n");

							if (p.dFast == true)
								nextState = State.ROTATE_FAST;
							else
								nextState = State.ROTATE_SLOW;
						}
					}
					break;

				case ROTATE_SLOW:
					
					p.calcErrors();
					if (Math.abs(p.errorAngle) < Math.abs(p.SLOW_STRAIGHT_ANGLE))
					{
						nextState = State.GO_SLOW;
						break;
					}
					p.turnRobot(p.errorAngle, false);
					break;

				case ROTATE_FAST:

					p.calcErrors();
					if (Math.abs(p.errorAngle) < Math.abs(p.FAST_STRAIGHT_ANGLE))
					{
						nextState = State.GO_FAST;
						break;
					}
					p.turnRobot(p.errorAngle, true);
					break;

				case ROTATE_TO_ANGLE:

					double rotate_angle = p.calcAngleToWaypointTheta();
					if (Math.abs(rotate_angle) < Math.abs(p.SLOW_STRAIGHT_ANGLE))
					{
						nextState = State.STOP;
						break;
					}
					p.turnRobot(rotate_angle, false);
					break;

				case GO_SLOW:

					p.calcErrors();

					if (p.errorDist > (p.prev_errorDist+p.PREVDIST_BUFFER))
					{
						if(verbose)System.out.printf("Current Distance to Waypoint > Previous Distance to Waypoint + BufVal\n");
						if(verbose2)System.out.printf("PrevDist:%f , Dist:%f",p.prev_errorDist, p.errorDist);
						nextState = State.STOP;
						break;
					}
					if (p.errorDist < p.SLOWDEST_DIST)
					{
						if(verbose)System.out.printf("Current Distance to Waypoint < Slow Speed Destination Distance\n");
						nextState = State.STOP;
						break;
					}
					p.moveRobotStraight(p.errorAngle, false);
					break;

				case GO_FAST:

					p.calcErrors();

					if (p.errorDist > (p.prev_errorDist+p.PREVDIST_BUFFER))
					{
						if(verbose)System.out.printf("......Current Distance to Waypoint > Previous Distance to Waypoint + BufVal\n");
						nextState = State.STOP;
						break;
					}
					if (p.errorDist < p.FASTDEST_DIST)
					{
						if(verbose)System.out.printf("Current Distance to Waypoint < Fast Speed Destination Distance\n");
						nextState = State.STOP;
						break;							
					}
					p.moveRobotStraight(p.errorAngle, true);
					break;
			}

			p.prev_errorDist = p.errorDist;
			prevState = state;
			state = nextState;

			try {
 				Thread.sleep(10);
			}
			catch(Exception e) {}

	}

	public void printState()
	{
		if (printcount++ > 40)
		{
			printcount = 0;

			if (state != prevState) System.out.println();
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
			else if (state == State.GO_SLOW)
				statestring = "GO_SLOW";
			else
				statestring = "UNKNOWN";
			System.out.println("Current State is: " + statestring);
			System.out.printf("Error Angle: %f, PreviousError Distance: %f, Error Distance: %f\n", p.errorAngle, p.prev_errorDist, p.errorDist);
			System.out.printf("Current position, X:%f, Y:%f, T:%f\n",p.cXYT[0],p.cXYT[1],p.cXYT[2]);
		}
	}
}
