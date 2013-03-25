package finallab;

import java.util.*;
import april.util.*;
import april.jmat.*;

import java.io.*;


public class Projectile
{

	ArrayList<double[]> ball_array = new ArrayList<double[]>();

	int num_pos;
	int num_balls=0;
	double starttime;
	double[] v_not = new double[3]; //x,y,z initial velocities, used in model

	boolean can_calib = false;
	boolean released = false;



	Projectile()
	{
		num_pos = 3;
	}

	Projectile(int Num_Pos)
	{
		num_pos = Num_Pos;
	}

	public void AddMeasurement(double[] xyzt)
	{
		ball_array.add(xyzt);
		num_balls++;
		
		if (!can_calib && (num_balls >= num_pos))
		{
			starttime = ball_array.get(num_balls)[3];
			can_calib = true;
		}
			
		if (!released && can_calib)
		{
			UpdateVnot();
		}
		 	
	
	}

	public void UpdateVnot()
	{
		double timediff = ball_array.get(num_balls-1)[3] - ball_array.get(num_balls-2)[3];
		v_not[0] = (ball_array.get(num_balls-1)[0] - ball_array.get(num_balls-2)[0]) / timediff;
		v_not[1] = (ball_array.get(num_balls-1)[1] - ball_array.get(num_balls-2)[1]) / timediff;
		v_not[2] = (ball_array.get(num_balls-1)[2] - ball_array.get(num_balls-2)[2]) / timediff;		
	}

	public static void main(String[] args) throws Exception
	{


	}
}
