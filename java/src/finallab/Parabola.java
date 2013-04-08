package finallab;

import java.util.*;
import java.io.*;


public class Parabola 
{
	//boolean valid;
	int first_ball;
	double error;
	double starttime;
	double land_time;
	double[] pred_landing;
	double[] parabola; //6x1 matrix with projectile parameters

	Parabola()
	{
		parabola = new double[6];
		pred_landing = new double[3];
		//valid = false;
		first_ball = 0;
	}

	public void updateParams(double[] params)
	{
		for (int i=0; i<6; i++)
		{
			parabola[i] = params[i];
		}
	}

	public void printParabola()
	{
		// if (valid)
		// 	System.out.printf("valid\n");
		// else
		// 	System.out.printf("valid\n");

		System.out.printf("first_ball:%d\n",first_ball);
		System.out.printf("error:%f\n",error);
		System.out.printf("starttime:%f\n",starttime);
		System.out.printf("land_time:%f\n",land_time);
		for (int i=0; i<3; i++)
			System.out.printf("landing%d: %f\n",i,pred_landing[i]);

	}

}