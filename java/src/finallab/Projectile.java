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

 
	LCM lcm;
	ArrayList<double[]> balls;
	ArrayList<double[]> pballs;
	ArrayList<double[]> fballs;
	int fake_index=0;
	boolean fake = true;		

	int num_meas;				//number of measurements used in prediction
	double starttime;
	double[] v_not; 			//x,y,z initial velocities, used in model
	double g = 9.806/(1000000000*1000000000); //g in meters/nanosecond squared
	BallStatus state;
	boolean verbose;
	final boolean DEFAULT_RELEASED = false;
	boolean released = false;

	JFrame jf;
	VisWorld vw;
	VisLayer vl;
	VisCanvas vc;
	ParameterGUI pg;	

	
	Projectile()
	{
		//vis initializations
		jf = new JFrame("RobotGUI");
		vw = new VisWorld();
		vl = new VisLayer(vw);
		vc = new VisCanvas(vl);
		pg = new ParameterGUI();
		pg.addCheckBoxes("Released?", "Released", DEFAULT_RELEASED);
		//pg.addCheckBoxes("dispConv", "Display Exploded Wall Map", DEFAULT_DISP_CONV);

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
			
		//projectile initializations
		balls = new ArrayList<double[]>();
		pballs = new ArrayList<double[]>();
		v_not = new double[3];
		state = BallStatus.WAIT; 

		verbose = true;

		if (fake)
		{
			fballs = new ArrayList<double[]>();
			createFakeData();
		}		
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_BALL"))
			{
				ball_t curr_ball = new ball_t(dins);
				
				double[] xyzt = new double[4];
				xyzt[0] = curr_ball.x;
				xyzt[1] = curr_ball.y;
				xyzt[2] = curr_ball.z;
				xyzt[3] = curr_ball.nanoTime;
				
				if(verbose)System.out.printf("num balls: %d\n", balls.size());

				if(fake)
					if (fake_index > 10)
						return;

				if (state == BallStatus.WAIT) //waits for 3 balls
				{
					if (balls.size() >=3)
					{
						state = BallStatus.IN_HAND;
						if (!fake)
							starttime = curr_ball.nanoTime;
						else
							starttime = fballs.get(fake_index)[3];
					}
					else
						if (fake)
						{
							balls.add(fballs.get(fake_index));
							fake_index++;
						}
						else
							balls.add(xyzt);
				}

				if (state == BallStatus.IN_HAND)
				{
					if (DetermineReleased())
					{
						state = BallStatus.RELEASED;
						if (!fake)
							starttime = curr_ball.nanoTime;
						else
							starttime = fballs.get(fake_index)[3];
						//GeneratePrediction();
					}

					balls.set(0, balls.get(1));
					balls.set(1, balls.get(2));


					if (fake)
					{
						balls.set(2, fballs.get(fake_index));
						fake_index++;
					}
					else
						balls.set(2, xyzt);

				}
				else if (state == BallStatus.RELEASED)
				{
					balls.add(xyzt);
				}

				DrawBalls();

		 	}
		 }
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public boolean DetermineReleased()
	{

		//update the velocity between balls at t-2 and t-1
		double prev_dt = balls.get(1)[3] - balls.get(0)[3];
		v_not[0] = (balls.get(1)[0] - balls.get(0)[0]) / prev_dt;
		v_not[1] = (balls.get(1)[1] - balls.get(0)[1]) / prev_dt;
		v_not[2] = (balls.get(1)[2] - balls.get(0)[2]) / prev_dt;		

		double cur_dt = balls.get(2)[3] - balls.get(1)[3];
		double[] predict_loc = Predict(cur_dt, 2); //predict location of ball 2 (first potentially released ball)
		
		double[] errors = new double[3];
		errors[0] = predict_loc[0]-balls.get(2)[0];
		errors[1] = predict_loc[1]-balls.get(2)[1];
		errors[2] = predict_loc[2]-balls.get(2)[2];

		if (verbose)
		{
			if (fake) System.out.printf("fakeindex: %d\n",fake_index);
			System.out.printf("zeroballx: %f\n",balls.get(0)[0]);
			System.out.printf("zerobally: %f\n",balls.get(0)[1]);
			System.out.printf("zeroballz: %f\n",balls.get(0)[2]);
			System.out.printf("oneballx: %f\n",balls.get(1)[0]);
			System.out.printf("onebally: %f\n",balls.get(1)[1]);
			System.out.printf("oneballz: %f\n",balls.get(1)[2]);
			System.out.printf("recentballx: %f\n",balls.get(2)[0]);
			System.out.printf("recentbally: %f\n",balls.get(2)[1]);
			System.out.printf("recentballz: %f\n",balls.get(2)[2]);
			System.out.printf("v_not[x]: %f\n",v_not[0]);
			System.out.printf("predict_loc[x]: %f\n",predict_loc[0]);
			System.out.printf("errors[x]: %f\n\n",errors[0]);
			System.out.printf("v_not[y]: %f\n",v_not[1]);
			System.out.printf("predict_loc[y]: %f\n",predict_loc[1]);
			System.out.printf("errors[y]: %f\n\n",errors[1]);
			System.out.printf("v_not[z]: %f\n",v_not[2]);
			System.out.printf("predict_loc[z]: %f\n",predict_loc[2]);
			System.out.printf("errors[z]: %f\n",errors[2]);						
		}

		double[] position = new double[3];
		position[0] = balls.get(2)[0];
		position[1] = balls.get(2)[1];
		position[2] = balls.get(2)[2];

		double e = Math.abs(predict_loc[0]-position[0])+Math.abs(predict_loc[1]-position[1])+Math.abs(predict_loc[2]-position[2]);
		System.out.printf("e:%f\n\n",e) ;
		//if (LinAlg.distance(predict_loc, balls.get(2)) < 0.2)
		if (e>0.5)
			return true;
		else
			return false;

	}

	public void DrawBalls()
	{

		double ball_radius = 0.04; 
		VisWorld.Buffer vb = vw.getBuffer("Predicted Balls");
		double[] shift = new double[3];

		for (int i=0; i<balls.size(); i++)
		{
			//shift = Predict(balls.get(i)[3] - starttime, i);
			//VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			//vb.addBack(new VisChain(LinAlg.translate(shift[0],shift[1],shift[2]),ball));
			
			VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			vb.addBack(new VisChain(LinAlg.translate(balls.get(i)[0],balls.get(i)[2],balls.get(i)[1]),ball));			
		}
/*
		for (int i=0; i<pballs.size(); i++)
		{
			shift = Predict(pballs.get(i)[3] - starttime, i);
			VzSphere pball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			vb.addBack(new VisChain(LinAlg.translate(shift[0],shift[1],shift[2]),pball));
		}*/

		vb.addBack(new VzAxes());

		vb.swap();

	}

	public void GeneratePrediction()
	{

		double time_aloft = 3; //need to solve for this
		int num_plotted = 20;

		for (int i=1; i<num_plotted; i++)
		{
			pballs.add(Predict((time_aloft/num_plotted)*(double)i, 1)); //use last held position as initial position
		}
	}

	public double[] Predict(double dt, int ballindex)
	{
		double[] predict_loc = new double[3]; 
		//predict_loc[0] = balls.get(ballindex)[0] + (v_not[0] * dt); //deltaX = Vo,x * dt
		predict_loc[1] = balls.get(ballindex)[1] + (v_not[1] * dt) - 0.5*g*dt*dt;
		predict_loc[1] = balls.get(ballindex)[1] + (v_not[1] * dt);
		predict_loc[2] = balls.get(ballindex)[2] + (v_not[2] * dt); 	
		return predict_loc;
	}

	public void createFakeData()
	{
		double[][] data = new double[20][4];

		double nano = 1000000000;

		data[0][0] = 0;  //x 
		data[0][1] = 2;  //y
		data[0][2] = 5;  //z
		data[0][3] = 0.1*nano;
		fballs.add(data[0]);

		data[1][0] = 0;  //x 
		data[1][1] = 2.5;  //y
		data[1][2] = 5;  //z
		data[1][3] = 0.2*nano;
		fballs.add(data[1]);

		data[2][0] = 0;  //x 
		data[2][1] = 2;  //y
		data[2][2] = 5;  //z
		data[2][3] = 0.3*nano;
		fballs.add(data[2]);

		data[3][0] = 0;  //x 
		data[3][1] = 2;  //y
		data[3][2] = 5;  //z
		data[3][3] = 0.4*nano;
		fballs.add(data[3]);

		data[4][0] = 0.5;  //x 
		data[4][1] = 2.4019;  //y
		data[4][2] = 5;  //z
		data[4][3] = 0.5*nano;
		fballs.add(data[4]);

		data[5][0] = 1;  //x 
		data[5][1] = 2.6078;  //y
		data[5][2] = 5;  //z
		data[5][3] = 0.6*nano;
		fballs.add(data[5]);

		data[6][0] = 1.5;  //x 
		data[6][1] = 2.6175;  //y
		data[6][2] = 5;  //z
		data[6][3] = 0.7*nano;
		fballs.add(data[6]);

		data[7][0] = 2.0;  //x 
		data[7][1] = 2.431;  //y
		data[7][2] = 5;  //z
		data[7][3] = 0.8*nano;
		fballs.add(data[7]);

		data[8][0] = 2.5;  //x 
		data[8][1] = 2.0485;  //y
		data[8][2] = 5;  //z
		data[8][3] = 0.9*nano;
		fballs.add(data[8]);

		data[9][0] = 3;  //x 
		data[9][1] = 1.4698;  //y
		data[9][2] = 5;  //z
		data[9][3] = 1*nano;
		fballs.add(data[9]);

		data[10][0] = 3.5;  //x 
		data[10][1] = 0.69506;  //y
		data[10][2] = 5;  //z
		data[10][3] = 0.7*nano;
		fballs.add(data[10]);

	}

	public static void main(String[] args) throws Exception
	{
		Projectile p = new Projectile();
	}
}













