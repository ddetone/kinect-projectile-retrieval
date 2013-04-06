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
	ArrayList<double[]> balls;
	ArrayList<double[]> pballs;
	ArrayList<double[]> fballs; 			//fake points
	ArrayList<double[]> landings;
	Parabola[] bounces;
	double[] v_not; 						//x,y,z initial velocities, used in model
	double starttime;						//time when ball is released
	
	final double nano = 1000000000;
	final double g = 9.806; 				//g in meters/second squared
	final double ball_radius = 0.06; 		//must be in meters
	final double error_thresh = 0.25;
	final int num_bounces = 4;
	final boolean DEFAULT_RELEASED = false;	//used in debugging
	final boolean fake = true;
	final boolean verbose = true;

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
		pg.addCheckBoxes("Released?", "Released", DEFAULT_RELEASED);

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
		//bounces = new Parabola()[num_bounces];

		v_not = new double[3];
		state = BallStatus.WAIT; 
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

				ball_t in_ball = new ball_t(dins);
				double[] xyzt = new double[4];
				xyzt[0] = in_ball.x;
				xyzt[1] = in_ball.z; //z is height
				xyzt[2] = in_ball.y; //y is lateral position from camera
				xyzt[3] = in_ball.nanoTime / nano;

				addBall(xyzt);

				PrintState();
				if (balls.size() >= 3) //wait for 3 balls
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
					CalculateParabola();
		
				}

				DrawBalls();

		 	}
		 }
		catch (IOException e)
		{
			e.printStackTrace();
		}
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

	//return false if not yet released
	public boolean CalculateParabola()
	{
		ArrayList<double[]> data = new ArrayList<double[]>();
		boolean error_ok = false;
		double error;

		System.out.printf("num_balls:%d\n",num_balls);
		//need at least three balls for release determination
		data.add(balls.get(num_balls-1));
		data.add(balls.get(num_balls-2));
		

		for (int i=num_balls-3; i>=0 ; i--)
		{
			data.add(balls.get(i));
			
		}

		if (solveLinReg(data))
			error_ok = true;

		//wrong
		return false;
	}

	public boolean solveLinReg(ArrayList<double[]> correspondences)
	{
		//Solves for x in, Ax=B
		int num_corr = correspondences.size();
		double error = 0;
		double[][] A = new double[3*num_corr][6];
		double[] B = new double[3*num_corr];
		double[] x = new double[6];

		for (int i=0; i<correspondences.size(); i++)
		{
			double t = correspondences.get(i)[3];
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
		if (verbose)
		{
			LinAlg.print(A);
			LinAlg.print(B);
		}

		//x = ((inv(AtA))At)B

		double[][] AtA = LinAlg.matrixAtB(A,A);
		double[][] invAtA = LinAlg.inverse(AtA);
		double[][] invAtAAt = LinAlg.matrixABt(invAtA,LinAlg.transpose(A));
		x = LinAlg.matrixAB(invAtAAt,B);

		if (verbose)
		{
			// System.out.printf("Ata\n");
			// LinAlg.print(AtA);
			// System.out.printf("invAtA\n");
			// LinAlg.print(invAtA);
			// System.out.printf("invAtAA\n");
			// LinAlg.print(invAtAAt);
			System.out.printf("x\n");
			LinAlg.print(x);
		}

		double time_aloft = 2;
		for (int i=0; i<20; i++)
		{
			double[] pred = new double[3];
			double dt = balls.get(num_balls-1)[3] + (time_aloft/20)*i;
			System.out.printf("dt is:%f\n",dt);
			pred[0] = x[0] + (x[1] * dt); //deltaX = Vo,x * dt
			pred[1] = x[2] + (x[3] * dt);
			pred[2] = x[4] + (x[5] * dt) - 0.5*g*dt*dt; 	
			pballs.add(pred);	
		} 
		

		//need to calculate errors

		//wrong
		return false;
	}
/*
	public boolean DetermineReleased()
	{

		//update the velocity between balls at t-2 and t-1
		double prev_dt = balls.get(1)[3] - balls.get(0)[3];
		v_not[0] = (balls.get(1)[0] - balls.get(0)[0]) / prev_dt;
		v_not[1] = (balls.get(1)[1] - balls.get(0)[1]) / prev_dt;
		v_not[2] = (balls.get(1)[2] - balls.get(0)[2]) / prev_dt;		

		double cur_dt = balls.get(2)[3] - balls.get(1)[3];
		double[] predict_loc = Predict(cur_dt, 1); //predict location of ball 1 (first potentially released ball)
		
		double[] errors = new double[3];
		errors[0] = predict_loc[0]-balls.get(2)[0];
		errors[1] = predict_loc[1]-balls.get(2)[1];
		errors[2] = predict_loc[2]-balls.get(2)[2];

		if (verbose)
		{
			if (fake) System.out.printf("fake_index: %d\n",fake_index);
			System.out.printf("zeroballx: %f\n",balls.get(0)[0]);
			System.out.printf("zerobally: %f\n",balls.get(0)[1]);
			System.out.printf("zeroballz: %f\n",balls.get(0)[2]);
			System.out.printf("zeroballt: %f\n",balls.get(0)[3]);
			System.out.printf("oneballx: %f\n",balls.get(1)[0]);
			System.out.printf("onebally: %f\n",balls.get(1)[1]);
			System.out.printf("oneballz: %f\n",balls.get(1)[2]);
			System.out.printf("oneballt: %f\n",balls.get(1)[3]);
			System.out.printf("recentballx: %f\n",balls.get(2)[0]);
			System.out.printf("recentbally: %f\n",balls.get(2)[1]);
			System.out.printf("recentballz: %f\n",balls.get(2)[2]);
			System.out.printf("recentballt: %f\n",balls.get(2)[3]);
			System.out.printf("prev_dt: %f\n", prev_dt);
			System.out.printf("cur_dt: %f\n", cur_dt);
			System.out.printf("v_not[x] (m/s): %f\n",v_not[0]);
			System.out.printf("predict_loc[x]: %f\n",predict_loc[0]);
			System.out.printf("errors[x]: %f\n",errors[0]);
			System.out.printf("v_not[y] (m/s): %f\n",v_not[1]);
			System.out.printf("predict_loc[y]: %f\n",predict_loc[1]);
			System.out.printf("errors[y]: %f\n",errors[1]);
			System.out.printf("v_not[z] (m/s): %f\n",v_not[2]);
			System.out.printf("predict_loc[z]: %f\n",predict_loc[2]);
			System.out.printf("errors[z]: %f\n",errors[2]);						
		}

		double[] position = new double[3];
		position[0] = balls.get(2)[0];
		position[1] = balls.get(2)[1];
		position[2] = balls.get(2)[2];

		//double e = Math.abs(predict_loc[0]-position[0])+Math.abs(predict_loc[1]-position[1])+Math.abs(predict_loc[2]-position[2]);
		double e = LinAlg.distance(predict_loc,position);

		System.out.printf("e:%f\n\n",e) ;
		//if (LinAlg.distance(predict_loc, balls.get(2)) < 0.2)
		if (e<0.25)
			return true;
		else
			return false;

	}
*/
	public void DrawBalls()
	{

		
		VisWorld.Buffer vb = vw.getBuffer("Predicted Balls");
		double[] shift = new double[3];

		for (int i=0; i<balls.size(); i++)
		{
			VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			vb.addBack(new VisChain(LinAlg.translate(balls.get(i)[0],balls.get(i)[1],balls.get(i)[2]),ball));			
		}

		for (int i=0; i<pballs.size(); i++)
		{
			VzSphere pball = new VzSphere(ball_radius, new VzMesh.Style(Color.green));
			vb.addBack(new VisChain(LinAlg.translate(pballs.get(i)[0],pballs.get(i)[1],pballs.get(i)[2]),pball));			
		}

		pballs.clear();

		//vb.addBack(new VzLines(new VisVertexData(pballs), 1, new VzLines.Style(Color.BLUE, 2)));	

/*
		if (landings.size() > 0)
		{
			VzCylinder land1 = new VzCylinder(0.15, 0.01, new VzMesh.Style(Color.green));
			vb.addBack(new VisChain(LinAlg.translate(landings.get(0)[0],landings.get(0)[1],landings.get(0)[2]), land1));
		}
*/
		vb.addBack(new VzAxes());
		vb.swap();

	}
	/*
	public void GeneratePrediction()
	{

		double[] cur_v_not = new double[3];

        for (int i=0; i<4; i++)
        {
        	if (i==0)
        		cur_v_not = v_not;

			double a = -0.5*g;
			double b = cur_v_not[2];
			double c = balls.get(1)[2]-ball_radius; //last in hand position
			//I subtracted ball_radius because we are actually solving when the bottom of the ball
			//hits the ground, not the center, which is approximated by the kinect sensor

			b = b / a;
			c = c / a;

			double discriminant = b*b - 4.0*c;
	        double sqroot =  Math.sqrt(discriminant);

	        double root1 = (-b + sqroot) / 2.0;
	        double root2 = (-b - sqroot) / 2.0;
	        double time_aloft = 0;

	        if (root1 > 0)
	        	time_aloft = root1;
	        else if (root2 > 0)
	        	time_aloft = root2;

	        cur_landing[0] = cur_v_not[0]*time_aloft + balls.get(1)[0];
	        cur_landing[1] = cur_v_not[1]*time_aloft + balls.get(1)[1];
	        cur_landing[2] = ball_radius;

	        if (verbose) 
	        {
		        System.out.printf("a:%f, b:%f, c:%f\n",a,b,c);
		        System.out.printf("discriminant:%f, sqroot:%f\n",discriminant,sqroot);
				System.out.printf("root1:%f root2:%f time_aloft:%f\n", root1, root2, time_aloft);
				System.out.printf("cur_landingX:%f, cur_landingY:%f cur_landingZ:%f\n\n", cur_landing[0],
					 cur_landing[1], cur_landing[2]);
			}

			landings.add(cur_landing);

			int num_plotted = 20;
			for (int j=1; j<num_plotted; j++)
			{
				pballs.add(Predict((time_aloft/num_plotted)*(double)j, 1)); //use last held position as initial position
			}
		}
	}

	public double[] Predict(double dt, int ballindex)
	{
		double[] predict_loc = new double[3]; 
		predict_loc[0] = balls.get(ballindex)[0] + (v_not[0] * dt); //deltaX = Vo,x * dt
		predict_loc[1] = balls.get(ballindex)[1] + (v_not[1] * dt);
		predict_loc[2] = balls.get(ballindex)[2] + (v_not[2] * dt) - 0.5*g*dt*dt; 	
		return predict_loc;
	}
*/
	public void createFakeData()
	{
		double[][] data = new double[20][4];

		//double nano = 1000000000;

		data[0][0] = 0;  //x 
		data[0][1] = 1;
		data[0][2] = 2.02;  //z
		data[0][3] = 0.4;
		fballs.add(data[0]);

		data[1][0] = 0.5;  //x 
		data[1][1] = 1.0;  //y
		data[1][2] = 2.451; //z
		data[1][3] = 0.5;
		fballs.add(data[1]);

		data[2][0] = 1;  //x 
		data[2][1] = 1.0;  //y
		data[2][2] = 2.904; //z
		data[2][3] = 0.6;
		fballs.add(data[2]);

		data[3][0] = 1.5;  //x 
		data[3][1] = 1.0;  //y
		data[3][2] = 3.059; //z
		data[3][3] = 0.7;
		fballs.add(data[3]);

		data[4][0] = 2.0;  //x 
		data[4][1] = 1.0;  //y
		data[4][2] = 3.2155; //z
		data[4][3] = 0.8;
		fballs.add(data[4]);

		data[5][0] = 2.5;  //x 
		data[5][1] = 1.0;
		data[5][2] = 3.27;  //z
		data[5][3] = 0.9*nano;
		fballs.add(data[5]);

		data[6][0] = 3;     //x 
		data[6][1] = 0.98;  //y
		data[6][2] = 3.234; //z
		data[6][3] = 1;
		fballs.add(data[6]);

		data[7][0] = 3.5;    //x 
		data[7][1] = 1.0;    //y
		data[7][2] = 3.0975; //z
		data[7][3] = 1.1;
		fballs.add(data[7]);

		data[8][0] = 3.95;  //x 
		data[8][1] = 1.0;   //y
		data[8][2] = 2.862; //z
		data[8][3] = 1.2;
		fballs.add(data[8]);

		data[9][0] = 4.5;    //x 
		data[9][1] = 1.04;   //y
		data[9][2] = 2.5286; //z
		data[9][3] = 1.3;
		fballs.add(data[9]);

		data[10][0] = 5.03;  //x 
		data[10][1] = 1.0;   //y
		data[10][2] = 2.097; //z
		data[10][3] = 1.4;
		fballs.add(data[10]);

		data[11][0] = 5.5;  //x 
		data[11][1] = 1.0;  //y
		data[11][2] = 1.56; //z
		data[11][3] = 1.49;
		fballs.add(data[11]);

		data[12][0] = 6;     //x 
		data[12][1] = 1.03;  //y
		data[12][2] = 0.939; //z
		data[12][3] = 1.6;
		fballs.add(data[12]);

		data[13][0] = 6.5;  //x 
		data[13][1] = 1;    //y
		data[13][2] = 0.21; //z
		data[13][3] = 1.71;
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

