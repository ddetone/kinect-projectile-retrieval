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
	int num_meas;				//number of measurements used in prediction
	int ball_i;
	double starttime;
	double[] v_not; 			//x,y,z initial velocities, used in model
	BallStatus state;
	boolean verbose;


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
		ball_i=-1;
		v_not = new double[3];
	
		verbose = true;		
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
				xyzt[3] = curr_ball.utime;
				
				ball_i++;

				if (state == BallStatus.WAIT && (balls.size() >= 3)) //waits for 3 balls
				{
					state = BallStatus.IN_HAND;
					starttime = curr_ball.utime;						
				}

				if (state == BallStatus.IN_HAND)
				{
					if (DetermineReleased())
					{
						state = BallStatus.RELEASED;
						starttime = curr_ball.utime;
						GeneratePrediction();
					}
				}

				if (state == BallStatus.IN_HAND)
				{
					balls.set(0, balls.get(1));
					balls.set(1, balls.get(2));
					balls.set(2, xyzt);
				}
				else if (state == BallStatus.RELEASED)
				{
					balls.add(xyzt);
				}

				if (verbose) {
					for (int i=0; i<3; i++)
					{
						for (int j=0; j<4; j++)
						{
							System.out.printf("balls[%d][%d]:%f\n",i,j,balls.get(i)[j]);
							System.out.printf("num balls: %d", ball_i);
							System.out.printf("state:%d", state);
						}
					}
					System.out.println();
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
		double prev_dt = balls.get(ball_i-1)[3] - balls.get(ball_i-2)[3];
		v_not[0] = (balls.get(ball_i-1)[0] - balls.get(ball_i-2)[0]) / prev_dt;
		v_not[1] = (balls.get(ball_i-1)[1] - balls.get(ball_i-2)[1]) / prev_dt;
		v_not[2] = (balls.get(ball_i-1)[2] - balls.get(ball_i-2)[2]) / prev_dt;		

		double cur_dt = balls.get(ball_i)[3] - balls.get(ball_i-1)[3];
		double[] predict_loc = Predict(cur_dt);
		
		double[] errors = new double[3];
		errors[0] = predict_loc[0]-balls.get(ball_i)[0];
		errors[1] = predict_loc[1]-balls.get(ball_i)[1];
		errors[2] = predict_loc[2]-balls.get(ball_i)[2];

		if (verbose)
		{
			for (int i=0; i<3; i++)
			{
				System.out.printf("v_not[%d]: %f\n",i,v_not[i]);
				System.out.printf("predict_loc[%d]: %f\n",i,predict_loc[i]);
				System.out.printf("errors[%d]: %f\n\n",i,errors[i]);
			}
		}

		if (LinAlg.distance(predict_loc, balls.get(num_meas)) < 0.2)
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
			shift = Predict(balls.get(i)[3] - starttime);
			VzSphere ball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			vb.addBack(new VisChain(LinAlg.translate(shift[0],shift[1],shift[2]),ball));
		}

		for (int i=0; i<pballs.size(); i++)
		{
			shift = Predict(pballs.get(i)[3] - starttime);
			VzSphere pball = new VzSphere(ball_radius, new VzMesh.Style(Color.red));
			vb.addBack(new VisChain(LinAlg.translate(shift[0],shift[1],shift[2]),pball));
		}

		vb.swap();

	}

	public void GeneratePrediction()
	{

		double time_aloft = 3; //need to solve for this
		int num_plotted = 20;

		for (int i=1; i<num_plotted; i++)
		{
			pballs.add(Predict((time_aloft/num_plotted)*i));
		}
	}

	public double[] Predict(double dt)
	{
		double[] predict_loc = new double[3];
		predict_loc[0] = balls.get(ball_i)[0] + (v_not[0] * dt); //deltaX = Vo,x * dt
		predict_loc[1] = balls.get(ball_i)[1] + (v_not[1] * dt) - 0.5*(9806000000000f)*dt*dt;
		predict_loc[2] = balls.get(ball_i)[2] + (v_not[2] * dt); 	
		return predict_loc;
	}

	public static void main(String[] args) throws Exception
	{
		Projectile p = new Projectile();
	}
}













