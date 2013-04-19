package finallab;

import java.io.*;
import java.util.*;

public class PathStateMachine
{
	public enum State {
		STOP, ROTATE_FAST, ROTATE_SLOW, ROTATE_TO_ANGLE, GO_FAST, GO_MED, GO_SLOW
	}

	volatile State state = State.STOP;
	State nextState = State.STOP;
	State prevState = State.STOP;
	PathFollower p = null;
	boolean verbose = true;
	boolean verbose2 = false;
	int printcount = 0;

	PathStateMachine() {}

	public void addParent(PathFollower _parent)
	{
		p = _parent;		
	}

	public void stateMachine()
	{
		if (p == null)
			return;

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
				if(verbose)System.out.printf("Rotate Angle Error:%f\n", rotate_angle);
				if (Math.abs(rotate_angle) < Math.abs(p.SLOW_STRAIGHT_ANGLE))
				{
					nextState = State.STOP;
					break;
				}
				p.turnRobot(rotate_angle, false);
				
				/*try {
					Thread.sleep(100);
				}
				catch(Exception e) {}*/

				break;

			case GO_SLOW:

				p.calcErrors();

				// if (p.errorDist > (p.prev_errorDist+p.PREVDIST_BUFFER))
				// {
				// 	if(verbose)System.out.printf("Current Waypoint Dist > Prev Dist to Waypoint + BufVal\n");
				// 	if(verbose2)System.out.printf("PrevDist:%f , Dist:%f",p.prev_errorDist, p.errorDist);
				// 	nextState = State.STOP;
				// 	break;
				// }
				if (p.errorDist < p.SLOWDEST_DIST)
				{
					if(verbose)System.out.printf("Current Waypoint Dist < Slow Speed Destination Dist\n");
					nextState = State.ROTATE_TO_ANGLE;
					break;
				}
				p.moveRobotStraight(p.errorAngle, p.SLOW_SPEED);

			case GO_MED:

				p.calcErrors();
				if (p.errorDist > (p.prev_errorDist+p.PREVDIST_BUFFER))
				{
					if(verbose)System.out.printf("Current Waypoint Dist > Prev Dist to Waypoint + BufVal\n");
					if(verbose2)System.out.printf("PrevDist:%f , Dist:%f",p.prev_errorDist, p.errorDist);
					nextState = State.STOP;
					break;
				}
				if (p.errorDist < p.MEDDEST_DIST)
				{
					if(verbose)System.out.printf("Current Waypoint Dist < Slow Speed Destination Dist\n");
					nextState = State.STOP;
					break;
				}
				p.moveRobotStraight(p.errorAngle, p.MED_SPEED);
				break;

			case GO_FAST:

				p.calcErrors();

				if (p.errorDist > (p.prev_errorDist+p.PREVDIST_BUFFER))
				{
					if(verbose)System.out.printf("Current Waypoint Dist > Prev Dist to Waypoint + BufVal\n");
					nextState = State.GO_MED;
					break;
				}
				if (p.errorDist < p.SLOW_DOWN_DIST)
				{
					if(verbose)System.out.printf("Slowing Down\n");
					nextState = State.GO_MED;
					break;							
				}
				p.moveRobotStraight(p.errorAngle, p.FAST_SPEED);
				break;
		}

		p.prev_errorDist = p.errorDist;
		prevState = state;
		state = nextState;

		/*
		try {
			Thread.sleep(10);
		}
		catch(Exception e) {}
		*/

	}

	public void printState()
	{
		if (printcount++ > -1 && prevState != State.STOP)
		{
			printcount = 0;

			if (nextState != state) System.out.println();
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
			System.out.printf("Error Angle: %f, PreviousError Distance: %f, Error Distance: %f\n", p.errorAngle, p.prev_errorDist, p.errorDist);
			System.out.printf("Current position, X:%f, Y:%f, T:%f\n",p.cXYT[0],p.cXYT[1],p.cXYT[2]);
		}
	}
}
