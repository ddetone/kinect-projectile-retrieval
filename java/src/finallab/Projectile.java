package finallab;

import java.util.*;
import java.io.*;
import java.awt.*;
import javax.swing.*;

import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;

import finallab.lcmtypes.*;

public class Projectile extends VisEventAdapter
{

	public enum BallStatus {
		WAIT, RELEASED, FINISHED, RESET
	}

	JFrame jf;
	VisWorld vw;
	VisLayer vl;
	VisCanvas vc;
	ParameterGUI pg;

	BallStatus state;
	ArrayList<double[]> balls;
	ArrayList<double[]> pballs; //dashed line for predicted path
	ArrayList<Parabola> bounces;


	final double nano = 1000000000;
	final double g = 9.806; 				//g in meters/second squared
	final double ball_radius = 0.03; 		//must be in meters
	final double DEFAULT_ERROR_THRESH = 0.01;
	final double bounce_factor = 0.75; 		//% bounce is retained
	final int num_bounces = 3;
	final int num_regression = 20; //the max number of recent balls used in regression
	final boolean DEFAULT_RESET = false;	//used in debugging
	final boolean verbose = true;
	final boolean verbose2 = false;

	int num_balls;
	int bounce_index;

	Projectile()
	{
		//vis initializations
		jf = new JFrame("Projectile");
		vw = new VisWorld();
		vl = new VisLayer(vw);
		vc = new VisCanvas(vl);
		pg = new ParameterGUI();
		pg.addCheckBoxes("Reset", "Reset? (double click the box)", DEFAULT_RESET);
		pg.addDoubleSlider("error_thresh","Error Threshold for Bounce Detection",0,0.025,DEFAULT_ERROR_THRESH);
		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI pg, String name)
			{
				if(name == "Reset")
				{
					if(pg.gb("Reset"))
						state = BallStatus.RESET;
				}
			}
		});

		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);
		jf.add(pg, BorderLayout.SOUTH);
		jf.setSize(800,600);
		jf.setVisible(true);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		vl.addEventHandler(this);
		vl.cameraManager.uiLookAt(new double[] {-2.66075, 1.22066, 1.70393 },
			new double[] {1.75367, -0.06226,  0.00000 },
			new double[] {0.33377, -0.09695,  0.93766 }, true);
		VisWorld.Buffer vb = vw.getBuffer("Ground");
		vb.addBack(new VisChain(LinAlg.translate(0,0,-0.025),new VzBox(30,30,0.05,new VzMesh.Style(Color.darkGray))));
		vb.swap();

		balls = new ArrayList<double[]>();
		pballs = new ArrayList<double[]>();

		//landings = new ArrayList<double[]>();
		bounces = new ArrayList<Parabola>();
		for (int i=0; i<num_bounces; i++)
		{
			Parabola emptyBounce = new Parabola();
			bounces.add(emptyBounce);
		}

		state = BallStatus.WAIT;	
		num_balls = 0;
		bounce_index = 0;

		/*
		friction = new double[3]; //a factor for air/ground friction to help with better prediction
		friction[0] = 0.02;	//subtract 2 cm per every 1 s
		friction[1] = 0.02;
		friction[2] = 0.02;
		*/

	}

	public void CheckBounce()
	{

		if (bounces.get(bounce_index).balls_in_parab < 3)
			return;

		double x[] = bounces.get(bounce_index).parabola;

		double[] actual = new double[3];
		double[] prediction = new double[3];

		actual[0] = balls.get(num_balls-1)[0];
		actual[1] = balls.get(num_balls-1)[1];
		actual[2] = balls.get(num_balls-1)[2];
		double dt = balls.get(num_balls-1)[3] - bounces.get(bounce_index).starttime;
		//double dt = correspondences.get(i)[3];
		prediction[0] = x[0] + (x[1] * dt); //deltaX = Vo,x * dt
		prediction[1] = x[2] + (x[3] * dt);
		prediction[2] = x[4] + (x[5] * dt) - 0.5*g*dt*dt;
		double e = LinAlg.squaredDistance(actual, prediction);
		if (verbose) System.out.printf("error:%f\n",e);
		double error_thresh = pg.gd("error_thresh");
		if (e > error_thresh)
		{
			System.out.printf("BOUNCE DETECTED\n");
			bounce_index++;
			if (bounce_index >= num_bounces)
			{
				System.out.printf("BOUNCE INDEX > NUM BOUNCES\n");
				state = BallStatus.FINISHED;
				return;
			}

			bounces.get(bounce_index).first_ball = num_balls-1;
			bounces.get(bounce_index).starttime = balls.get(num_balls-1)[3];
		}


	}

	public void CalculateParabola()
	{
		ArrayList<double[]> data = new ArrayList<double[]>();
		boolean is_parab = false;
		double error;

		System.out.printf("num_balls:%d\n",num_balls);
		//need at least three balls for release determination

		// data.add(balls.get(num_balls-1));
		// data.add(balls.get(num_balls-2));
		int num_reg_balls = Math.min(num_balls - bounces.get(bounce_index).first_ball, num_regression);

		if (verbose) System.out.printf("num_reg_balls:%d\n", num_reg_balls);

		if (num_reg_balls < 3)
			return;

		for (int i=1; (i <= num_reg_balls); i++)
			data.add(balls.get(num_balls-i));

		solveLinReg(data);

	}

	//Calculates linear regression for projectile
	public void solveLinReg(ArrayList<double[]> correspondences)
	{
		//Solves for x in, Ax=B
		int num_corr = correspondences.size();
		double[][] A = new double[3*num_corr][6];
		double[] B = new double[3*num_corr];
		double[] x = new double[6];

		
		int oldestball_index = bounces.get(bounce_index).first_ball;

		double starttime = balls.get(oldestball_index)[3];
		

		//the parameters for the regression are solved such that at a given starttime
		//the xyz displacements are equal to the x_0, y_0, z_0
		for (int i=0; i<correspondences.size(); i++)
		{
			double t = correspondences.get(i)[3] - starttime;
			A[i][0] = 1;
			A[i][1] = t;
			A[i+num_corr][2] = 1;
			A[i+num_corr][3] = t;
			A[i+num_corr*2][4] = 1;
			A[i+num_corr*2][5] = t;
			//A[i+num_corr*2][6] = t*t;
			B[i] = correspondences.get(i)[0];
			B[i+num_corr] = correspondences.get(i)[1];
			B[i+num_corr*2] = correspondences.get(i)[2] + 0.5*g*t*t;
		}

		double[][] AtA = LinAlg.matrixAtB(A,A);
		double[][] invAtA = LinAlg.inverse(AtA);
		double[][] invAtAAt = LinAlg.matrixABt(invAtA,A);
		x = LinAlg.matrixAB(invAtAAt,B);

		if (verbose2)
		{
			LinAlg.print(A);
			LinAlg.print(B);
			System.out.printf("x\n");
			LinAlg.print(x);

			ArrayList<Double> errors = new ArrayList<Double>();
			double[] prediction = new double[3];
			for (int i=0; i<correspondences.size(); i++)
			{
				double[] actual = new double[3];
				actual[0] = correspondences.get(i)[0];
				actual[1] = correspondences.get(i)[1];
				actual[2] = correspondences.get(i)[2];
				double dt = correspondences.get(i)[3] - starttime;
				//double dt = correspondences.get(i)[3];
				prediction[0] = x[0] + (x[1] * dt); //deltaX = Vo,x * dt
				prediction[1] = x[2] + (x[3] * dt);
				prediction[2] = x[4] + (x[5] * dt) - 0.5*g*dt*dt;
				double e = LinAlg.squaredDistance(actual, prediction);
				errors.add(e);
			}

			double error = 0;
			for (int i=0; i<errors.size(); i++)
			{
				error += errors.get(i);
			}
			error = error / errors.size();


			for (int i=0; i<errors.size(); i++)
				System.out.printf("Error at %d:%f\n",i,errors.get(i));
			System.out.printf("Average squared error:%f\n",error);

			bounces.get(bounce_index).error = error;
		}


			//update current bounce parameters
		bounces.get(bounce_index).starttime = starttime;
		bounces.get(bounce_index).updateParams(x);
		bounces.get(bounce_index).balls_in_parab++;
		CalculateLanding(bounce_index);
		if (verbose2) bounces.get(bounce_index).printParabola(bounce_index);
		CalculateNextParabolas();

		return;

	}

	public void CalculateNextParabolas()
	{

		for (int i=bounce_index+1; i<num_bounces; i++)
		{
			Parabola prevBounce = bounces.get(i-1);
			double lt = bounces.get(i-1).land_time;
			bounces.get(i).starttime = bounces.get(i-1).starttime + lt;

			double[] newx = new double[6];
			newx[0] = prevBounce.parabola[0] + prevBounce.parabola[1]*lt;
			newx[1] = prevBounce.parabola[1];
			newx[2] = prevBounce.parabola[2] + prevBounce.parabola[3]*lt;
			newx[3] = prevBounce.parabola[3];
			newx[4] = ball_radius;
			newx[5] = (prevBounce.parabola[5] - g*lt)*(-1*bounce_factor);
			bounces.get(i).updateParams(newx);
			
			CalculateLanding(i);

			if(verbose2)bounces.get(i).printParabola(i);
		}

	}

	//requires .parabola .starttime .firstball
	public void CalculateLanding(int bindex)
	{
		Parabola currentBounce = bounces.get(bindex);
		double a = -0.5*g;
			double b = currentBounce.parabola[5]; //z_vel
			double c = currentBounce.parabola[4] - ball_radius; //z_not - ball_radius

			b = b / a;
			c = c / a;

			double discriminant = b*b - 4.0*c;
			double sqroot =  Math.sqrt(discriminant);

			double root1 = (-b + sqroot) / 2.0;
			double root2 = (-b - sqroot) / 2.0;
			double nextroot = 0;
			double prevroot = 0;

			if (root1 > root2)
			{
				nextroot = root1;
				prevroot = root2;
			}
			else
			{
				nextroot = root2;
				prevroot = root1;
			}

			double land_time;
			land_time = nextroot;

			double[] cur_landing = new double[3];
			cur_landing[0] = currentBounce.parabola[1]*land_time + currentBounce.parabola[0];
			cur_landing[1] = currentBounce.parabola[3]*land_time + currentBounce.parabola[2];
			cur_landing[2] = ball_radius;

			if (verbose2) 
			{
				System.out.printf("a:%f, b:%f, c:%f\n",a,b,c);
				System.out.printf("discriminant:%f, sqroot:%f\n",discriminant,sqroot);
				System.out.printf("root1:%f root2:%f land_time:%f\n", root1, root2, land_time);
				System.out.printf("cur_landingX:%f, cur_landingY:%f cur_landingZ:%f\n\n", cur_landing[0],
					cur_landing[1], cur_landing[2]);
			}

			currentBounce.land_time = land_time;
			currentBounce.pred_landing = cur_landing;
			currentBounce.valid = true;        							

		}

		public void DrawBalls()
		{

			VisWorld.Buffer vb = vw.getBuffer("Predicted Balls");
			double[] shift = new double[3];

			for (int i=0; i<balls.size(); i++)
			{
				VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.green));
				vb.addBack(new VisChain(LinAlg.translate(balls.get(i)[0],balls.get(i)[1],balls.get(i)[2]),ball));			
			}


			for (int i=0; i<num_bounces; i++)
			{

				double land_time = bounces.get(i).land_time;
			//double starttime = bounces.get(i).starttime;
				double[] x = bounces.get(i).parabola;

				for (int j=0; j<20; j++)
				{
					double[] pred = new double[3];
					double dt = (land_time/20)*j;
				pred[0] = x[0] + (x[1] * dt); //deltaX = Vo,x * dt
				pred[1] = x[2] + (x[3] * dt);
				pred[2] = x[4] + (x[5] * dt) - 0.5*g*dt*dt; 	
				pballs.add(pred);	
			} 

			if (i%2 == 0)
				vb.addBack(new VzLines(new VisVertexData(pballs), 1, new VzLines.Style(Color.BLUE, 2)));
			else
				vb.addBack(new VzLines(new VisVertexData(pballs), 1, new VzLines.Style(Color.RED, 2)));

		}

		pballs.clear();

		for (int i=0; i<num_bounces; i++)
		{
			VzCylinder land1 = new VzCylinder(0.15, 0.01, new VzMesh.Style(Color.cyan));
			vb.addBack(new VisChain(LinAlg.translate(bounces.get(i).pred_landing[0],
				bounces.get(i).pred_landing[1], bounces.get(i).pred_landing[2]), land1));
		}

		vb.addBack(new VzAxes());
		vb.swap();

	}

	public void DrawBallsWithRobot(Point3D robotLoc, double[] xyzt)
	{

		VisWorld.Buffer vb = vw.getBuffer("Predicted Balls");
		double[] shift = new double[3];

		for (int i=0; i<balls.size(); i++)
		{
			VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.green));
			vb.addBack(new VisChain(LinAlg.translate(balls.get(i)[0],balls.get(i)[1],balls.get(i)[2]),ball));			
		}


		for (int i=0; i<num_bounces; i++)
		{

			double land_time = bounces.get(i).land_time;
			//double starttime = bounces.get(i).starttime;
			double[] x = bounces.get(i).parabola;

			for (int j=0; j<20; j++)
			{
				double[] pred = new double[3];
				double dt = (land_time/20)*j;
				pred[0] = x[0] + (x[1] * dt); //deltaX = Vo,x * dt
				pred[1] = x[2] + (x[3] * dt);
				pred[2] = x[4] + (x[5] * dt) - 0.5*g*dt*dt; 	
				pballs.add(pred);	
			} 

			if (i%2 == 0)
				vb.addBack(new VzLines(new VisVertexData(pballs), 1, new VzLines.Style(Color.BLUE, 2)));
			else
				vb.addBack(new VzLines(new VisVertexData(pballs), 1, new VzLines.Style(Color.RED, 2)));


		}

		

		/*
		for (int i=0; i<pballs.size(); i++)
		{
			VzSphere pball = new VzSphere(ball_radius, new VzMesh.Style(Color.green));
			vb.addBack(new VisChain(LinAlg.translate(pballs.get(i)[0],pballs.get(i)[1],pballs.get(i)[2]),pball));			
		}*/

		pballs.clear();

		for (int i=0; i<num_bounces; i++)
		{
			VzCylinder land1 = new VzCylinder(0.15, 0.01, new VzMesh.Style(Color.cyan));
			vb.addBack(new VisChain(LinAlg.translate(bounces.get(i).pred_landing[0],
				bounces.get(i).pred_landing[1], bounces.get(i).pred_landing[2]), land1));
		}
		double wheelRadius = 0.04;
		VzBox base = new VzBox(0.155,0.166,0.07, new VzMesh.Style(Color.red));
		VisObject vo_base = new VisChain(LinAlg.translate(0,0.06,0.10),base);

		VzBox cameraBase = new VzBox(0.05,0.01,0.04, new VzMesh.Style(Color.white));
		VisObject vo_cameraBase = new VisChain(LinAlg.translate(0,0,0.145),cameraBase);

		VzCylinder wheels = new VzCylinder(wheelRadius,0.01, new VzMesh.Style(Color.white));
		VisObject vo_wheels = new VisChain(LinAlg.rotateY(Math.PI/2),LinAlg.translate(-wheelRadius,0,0.09),wheels,LinAlg.translate(0,0,-0.18),wheels);

		double castorRad = 0.03;
		VzCylinder castor = new VzCylinder(castorRad,0.02, new VzMesh.Style(Color.black));
		VisObject vo_castor = new VisChain(LinAlg.rotateY(Math.PI/2), LinAlg.translate(-castorRad,0.115,0),castor);

		VisChain pandaBot = new VisChain();

		pandaBot.add(vo_base,vo_cameraBase,vo_wheels,vo_castor);

		//vb.addBack(new VzAxes());
		//vb.addBack(new VisChain(LinAlg.translate(xyt[0],xyt[1],0), LinAlg.rotateZ(xyt[2]-Math.PI/2),new VzTriangle(0.25,0.4,0.4,new VzMesh.Style(Color.GREEN))));
		vb.addBack(new VisChain(LinAlg.translate(robotLoc.x,robotLoc.y,0),pandaBot));

		VisChain path = new VisChain(LinAlg.translate(xyzt[0],0),LinAlg.rotateZ(xyzt[2]),LinAlg.translate(robotLoc.x,robotLoc.y), pandaBot);//new VzBox(xyzt[0], .1, .1));
		vb.addBack(path);

		vb.addBack(new VzAxes());
		vb.swap();

	}

	public void update(ball_t in_ball)
	{
		PrintState();
		if (state == BallStatus.FINISHED)
		{
			if (verbose) System.out.printf("Reset to continue\n");
			return;
		}

		PrintState();
		if (state == BallStatus.RESET) //reset everything
		{
			num_balls = 0;
			bounce_index = 0;
			bounces.clear();
			for (int i=0; i<num_bounces; i++)
			{
				Parabola emptyBounce = new Parabola();
				bounces.add(emptyBounce);
			}
			balls.clear();
			state = BallStatus.WAIT;
		}

		double[] xyzt = new double[4];
		xyzt[0] = in_ball.x;
		xyzt[1] = in_ball.z; //z is height
		xyzt[2] = in_ball.y; //y is lateral position from camera
		xyzt[3] = in_ball.nanoTime / nano;

		//bounds checking for ball locations
		if ((Math.abs(xyzt[0]) > 100) || (Math.abs(xyzt[0]) > 100) || (Math.abs(xyzt[0]) > 100))
			return;
		
		//add a ball
		balls.add(xyzt);
		num_balls++;

		PrintState();
		if (balls.size() >= 3 && state == BallStatus.WAIT) //wait for 3 balls
		{	
			state = BallStatus.RELEASED;
			bounces.get(0).first_ball = 0;
			bounces.get(0).starttime = balls.get(0)[3];
			bounces.get(0).balls_in_parab = 3;
		}

		PrintState();
		if (state == BallStatus.RELEASED)
		{
			if (num_balls >= 4)
				CheckBounce();
			if (state == BallStatus.FINISHED)
				return;

			CalculateParabola();
		}

		DrawBalls();
	}

	public void PrintState()
	{

		VisWorld.Buffer vb = vw.getBuffer("State");

		String statestring;
		if (state == BallStatus.WAIT)
			statestring = "WAIT";
		else if (state == BallStatus.RELEASED)
			statestring = "RELEASED";
		else if (state == BallStatus.FINISHED)
			statestring = "FINISHED";
		else if (state == BallStatus.RESET)
			statestring = "RESET";
		else
			statestring = "UNKNOWN";

		vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.CENTER, new VzText(VzText.ANCHOR.CENTER, 
			"<<sansserif-bold-16,white>>" + statestring)));
		
		vb.swap();
	}

	public ArrayList<Parabola> getParabolas()
	{
		return bounces;
	}

	public static void main(String[] args) throws Exception
	{
		Projectile p = new Projectile();
	}
}

