package finallab;

import java.util.*;
import java.io.*;
import java.awt.*;
import javax.swing.*;

import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;
import lcm.lcm.*;
import finallab.lcmtypes.*;

public class Projectile extends VisEventAdapter implements LCMSubscriber
{

	public enum BallStatus {
		WAIT, IN_HAND, RELEASED
	}

	JFrame jf;
	VisWorld vw;
	VisLayer vl;
	VisCanvas vc;
	ParameterGUI pg;
 
	LCM lcm;
	BallStatus state;
	BallStatus prevstate;
	ArrayList<double[]> balls;
	ArrayList<double[]> pballs;
	ArrayList<double[]> fballs; 			//fake points
	ArrayList<double[]> landings;
	ArrayList<Parabola> bounces;
	double[] friction;


	final double nano = 1000000000;
	final double g = 9.806; 				//g in meters/second squared
	final double ball_radius = 0.03; 		//must be in meters
	final double DEFAULT_ERROR_THRESH = 0.02;
	final double bounce_factor = 0.75; 		//60% bounce is retained
	final int num_bounces = 3;
	final boolean DEFAULT_RELEASED = false;	//used in debugging
	final boolean fake = false;
	final boolean verbose = true;
	final boolean verbose2 = false;

	//int fake_index;
	int num_balls;
	int bounce_index;

	Projectile()
	{
		//vis initializations
		jf = new JFrame("RobotGUI");
		vw = new VisWorld();
		vl = new VisLayer(vw);
		vc = new VisCanvas(vl);
		pg = new ParameterGUI();
		pg.addCheckBoxes("Released", "Released?", DEFAULT_RELEASED);
		pg.addDoubleSlider("error_thresh","Average Squared Error Threshold",0,0.25,DEFAULT_ERROR_THRESH);
		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI pg, String name)
			{
				if(name == "Released")
				{
					if(pg.gb("Released"))
						state = BallStatus.RELEASED;
					else
						state = BallStatus.IN_HAND;
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

		try {
		 	this.lcm = LCM.getSingleton();
			this.lcm.subscribe("6_BALL", this);
		} catch (Exception ex) {
          System.out.println("Exception: " + ex);
     	}
			
		balls = new ArrayList<double[]>();
		pballs = new ArrayList<double[]>();
		if (fake)
		{
			fballs = new ArrayList<double[]>();
			createFakeData();
		}		
		landings = new ArrayList<double[]>();
		bounces = new ArrayList<Parabola>();
		for (int i=0; i<num_bounces; i++)
		{
			Parabola emptyBounce = new Parabola();
			bounces.add(emptyBounce);
		}
		friction = new double[3]; //a factor for air/ground friction to help with better prediction
		friction[0] = 0.02;	//subtract 2 cm per every 1 s
		friction[1] = 0.02;
		friction[2] = 0.02;

		state = BallStatus.WAIT;
		prevstate = BallStatus.WAIT;
		//fake_index = 0;	
		num_balls = 0;
		bounce_index = 0;
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_BALL"))
			{

				if(fake && num_balls >= fballs.size())
					return;

				prevstate = state;
				ball_t in_ball = new ball_t(dins);
				double[] xyzt = new double[4];
				xyzt[0] = in_ball.x;
				xyzt[1] = in_ball.z; //z is height
				xyzt[2] = in_ball.y; //y is lateral position from camera
				xyzt[3] = in_ball.nanoTime / nano;

				if ((Math.abs(xyzt[0]) > 100) || (Math.abs(xyzt[0]) > 100) || (Math.abs(xyzt[0]) > 100))
					return;
				
				addBall(xyzt);

				PrintState();
				if (balls.size() >= 3 && state == BallStatus.WAIT) //wait for 3 balls
					state = BallStatus.IN_HAND;


				PrintState();
				if (state == BallStatus.IN_HAND)
				{
					if (CalculateParabola())
					{
						state = BallStatus.RELEASED;
						//update starttime of first parabola						
					}
				}

				PrintState();
				if (state == BallStatus.RELEASED)
				{
					if (prevstate == BallStatus.RELEASED)
						CalculateParabola();
					if (CheckBounce())
						bounce_index++;
				}

				DrawBalls();

		 	}
		 }
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public boolean CheckBounce()
	{
		if ((balls.get(num_balls-1)[2] > balls.get(num_balls-2)[2]) &&
			(balls.get(num_balls-2)[2] < balls.get(num_balls-3)[2]))
		{
			if (verbose) System.out.printf("BOUNCE\n");
			return true;
		}
		else
			return false;
	}

	//return false if not yet released
	public boolean CalculateParabola()
	{
		ArrayList<double[]> data = new ArrayList<double[]>();
		boolean is_parab = false;
		double error;

		System.out.printf("num_balls:%d\n",num_balls);
		//need at least three balls for release determination
		data.add(balls.get(num_balls-1));
		data.add(balls.get(num_balls-2));
		

		for (int i=num_balls-3; i>=0 ; i--)
		{
			boolean error_ok = true;
			data.add(balls.get(i));
			
			error_ok = solveLinReg(data, i);

			if (error_ok)
			{
				is_parab = true;
			}
			if (!error_ok && is_parab)
				break;
		}

		if (is_parab)
			return true;
		else
			return false;
	}

	//Calculates linear regression for projectile. if the error is less that error_thresh
	//it will update the current bounce[bounce_index] with the calculated parameters
	public boolean solveLinReg(ArrayList<double[]> correspondences, int oldestball_index)
	{
		//Solves for x in, Ax=B
		int num_corr = correspondences.size();
		double[][] A = new double[3*num_corr][6];
		double[] B = new double[3*num_corr];
		double[] x = new double[6];

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

		if (verbose)
		{
			LinAlg.print(A);
			LinAlg.print(B);
			System.out.printf("x\n");
			LinAlg.print(x);
		}

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

		if (verbose)
		{
			for (int i=0; i<errors.size(); i++)
				System.out.printf("Error at %d:%f\n",i,errors.get(i));
			System.out.printf("Average squared error:%f\n",error);
		}

		double error_thresh = pg.gd("error_thresh");

		if (error < error_thresh)
		{
			//update current bounce parameters
			bounces.get(bounce_index).starttime = starttime;
			bounces.get(bounce_index).error = error;
			bounces.get(bounce_index).first_ball = oldestball_index;
			bounces.get(bounce_index).updateParams(x);
			CalculateLanding(bounce_index);

			if(verbose2)bounces.get(bounce_index).printParabola(bounce_index);
			CalculateNextParabolas();

			return true;
		}
		else
			return false;
	}

	public void CalculateNextParabolas()
	{

		for (int i=bounce_index+1; i<num_bounces; i++)
		{
			double lt = bounces.get(i-1).land_time;
			bounces.get(i).starttime = bounces.get(i-1).starttime + lt;
			//bounces.get(i).starttime = lt;

			double[] newx = new double[6];
			newx[0] = bounces.get(i-1).parabola[0] + bounces.get(i-1).parabola[1]*lt;
			//newx[0] = bounces.get(i-1).parabola[0];
			newx[1] = bounces.get(i-1).parabola[1];
			newx[2] = bounces.get(i-1).parabola[2] + bounces.get(i-1).parabola[3]*lt;
			//newx[2] = bounces.get(i-1).parabola[2];
			newx[3] = bounces.get(i-1).parabola[3];
			newx[4] = ball_radius;
			//newx[5] = (bounces.get(i-1).parabola[5] - g*(lt - bounces.get(i-1).starttime))*(-1*bounce_factor);
			newx[5] = (bounces.get(i-1).parabola[5] - g*lt)*(-1*bounce_factor);
			bounces.get(i).updateParams(newx);
			
			CalculateLanding(i);

			if(verbose2)bounces.get(i).printParabola(i);
		}

	}

	//requires .parabola .starttime .firstball
	public void CalculateLanding(int bindex)
	{

			double a = -0.5*g;
			double b = bounces.get(bindex).parabola[5]; //z_vel
			double c = bounces.get(bindex).parabola[4] - ball_radius; //z_not - ball_radius

			b = b / a;
			c = c / a;

			double discriminant = b*b - 4.0*c;
	        double sqroot =  Math.sqrt(discriminant);

	        double root1 = (-b + sqroot) / 2.0;
	        double root2 = (-b - sqroot) / 2.0;
	        double nextroot = 0;
	        double prevroot = 0;

	        //if (root1 > bounces.get(bindex).starttime)
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
	        /*
	        if (bindex == 0)
	        {
	        	//double release_height = bounces.get(0).parabola[0] + bounces.get(0).parabola[1]*bounces.get(0).starttime;
	        	
	        	//need to subract off the time it takes for the ball to travel from release height to the ground given
	        	//the intial velocity in the z direction. 
	        	land_time = nextroot - (bounces.get(0).starttime - prevroot);

	        }
	        else //don't need to account for release height for these bounces
	        	land_time = nextroot + bounces.get(bindex-1).land_time;
		*/

	        double[] cur_landing = new double[3];
	        cur_landing[0] = bounces.get(bindex).parabola[1]*land_time + bounces.get(bindex).parabola[0];
	        cur_landing[1] = bounces.get(bindex).parabola[3]*land_time + bounces.get(bindex).parabola[2];
	        cur_landing[2] = ball_radius;

	        if (verbose) 
	        {
		        System.out.printf("a:%f, b:%f, c:%f\n",a,b,c);
		        System.out.printf("discriminant:%f, sqroot:%f\n",discriminant,sqroot);
				System.out.printf("root1:%f root2:%f land_time:%f\n", root1, root2, land_time);
				System.out.printf("cur_landingX:%f, cur_landingY:%f cur_landingZ:%f\n\n", cur_landing[0],
					 cur_landing[1], cur_landing[2]);
			}

			bounces.get(bindex).land_time = land_time;
			bounces.get(bindex).pred_landing = cur_landing;        							

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

		vb.addBack(new VzAxes());
		vb.swap();

	}

	public void addBall(double[] xyzt)
	{
		if (fake)
		{
			balls.add(fballs.get(num_balls));
			num_balls++;
		}
		else
		{
			balls.add(xyzt);
			num_balls++;
		}
	}

	public void createFakeData()
	{
		double[][] data = new double[20][4];
		double timeoffset = 5.47;

		//double nano = 1000000000;

		data[0][0] = 0;  //x 
		data[0][1] = 0;
		data[0][2] = 2;  //z
		data[0][3] = 0 + timeoffset;
		fballs.add(data[0]);

		data[1][0] = 1.5;  //x 
		data[1][1] = 1.0;  //y
		data[1][2] = 2.451; //z
		data[1][3] = 0.1 + timeoffset;
		fballs.add(data[1]);

		data[2][0] = 1;  //x 
		data[2][1] = 1.02;  //y
		data[2][2] = 2.904; //z
		data[2][3] = 0.2 + timeoffset;
		fballs.add(data[2]);

		data[3][0] = 1.5;  //x 
		data[3][1] = 1.0;  //y
		data[3][2] = 3.059; //z
		data[3][3] = 0.3 + timeoffset;
		fballs.add(data[3]);

		data[4][0] = 2.05;  //x 
		data[4][1] = 1.0;  //y
		data[4][2] = 3.2155; //z
		data[4][3] = 0.4 + timeoffset;
		fballs.add(data[4]);

		data[5][0] = 2.5;  //x 
		data[5][1] = 1.0;
		data[5][2] = 3.27;  //z
		data[5][3] = 0.5 + timeoffset;
		fballs.add(data[5]);

		data[6][0] = 3;     //x 
		data[6][1] = 0.98;  //y
		data[6][2] = 3.234; //z
		data[6][3] = 0.6 + timeoffset;
		fballs.add(data[6]);

		data[7][0] = 3.5;    //x 
		data[7][1] = 1.04;    //y
		data[7][2] = 3.0975; //z
		data[7][3] = 0.7 + timeoffset;
		fballs.add(data[7]);

		data[8][0] = 3.95;  //x 
		data[8][1] = 1.0;   //y
		data[8][2] = 2.862; //z
		data[8][3] = 0.8 + timeoffset;
		fballs.add(data[8]);

		data[9][0] = 4.5;    //x 
		data[9][1] = 1.04;   //y
		data[9][2] = 2.5286; //z
		data[9][3] = 0.9 + timeoffset;
		fballs.add(data[9]);

		data[10][0] = 5.03;  //x 
		data[10][1] = 1.0;   //y
		data[10][2] = 2.097; //z
		data[10][3] = 1.0 + timeoffset;
		fballs.add(data[10]);

		data[11][0] = 5.46;  //x 
		data[11][1] = 1.0;  //y
		data[11][2] = 1.56; //z
		data[11][3] = 1.1 + timeoffset;
		fballs.add(data[11]);

		data[12][0] = 6;     //x 
		data[12][1] = 1.03;  //y
		data[12][2] = 0.939; //z
		data[12][3] = 1.2 + timeoffset;
		fballs.add(data[12]);

		data[13][0] = 6.5;  //x 
		data[13][1] = 1;    //y
		data[13][2] = 0.21; //z
		data[13][3] = 1.3 + timeoffset;
		fballs.add(data[13]);

	}

	public void PrintState()
	{

		VisWorld.Buffer vb = vw.getBuffer("State");

		String statestring;
		if (state == BallStatus.WAIT)
			statestring = "WAIT";
		else if (state == BallStatus.IN_HAND)
			statestring = "IN_HAND";
		else if (state == BallStatus.RELEASED)
			statestring = "RELEASED";
		else
			statestring = "UNKNOWN";

		vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.CENTER, new VzText(VzText.ANCHOR.CENTER, 
							"<<sansserif-bold-16,white>>" + statestring)));
		
		vb.swap();
	}

	public static void main(String[] args) throws Exception
	{
		Projectile p = new Projectile();
	}
}

