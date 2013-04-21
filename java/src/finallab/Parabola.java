package finallab;

import java.util.*;
import java.io.*;

import april.util.*;
import april.jmat.*;


public class Parabola 
{
	volatile boolean valid;
	int first_ball;
	int balls_in_parab;
	double error; //Only calculates error if verbose2 is set, for optimizations
	double starttime;
	double land_time;
	double[] pred_landing;
	double[] pred_botlanding;
	double[] parabola; //6x1 matrix with projectile parameters

	Parabola()
	{
		parabola = new double[6];
		pred_landing = new double[3];
		pred_botlanding = new double[3];
		valid = false;
		first_ball = 0;
		balls_in_parab = 0;
	}

	public void updateParams(double[] params)
	{
		for (int i=0; i<6; i++)
		{
			parabola[i] = params[i];
		}
	}

	public void printParabola(int index)
	{
		// if (valid)
		// 	System.out.printf("valid\n");
		// else
		// 	System.out.printf("valid\n");

		System.out.printf("-----Parabola number: %d\n", index);
		LinAlg.print(parabola);
		System.out.printf("first_ball:%d\n",first_ball);
		System.out.printf("error:%f\n",error);
		System.out.printf("starttime:%f\n",starttime);
		System.out.printf("land_time:%f\n",land_time);
		for (int i=0; i<3; i++)
		{
			System.out.printf("Pred landing%d: %f\n",i,pred_landing[i]);
			System.out.printf("Pred bot landing%d: %f\n",i,pred_botlanding[i]);
		}

	}

}