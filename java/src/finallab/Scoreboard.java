package finallab;

import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.io.*;
import java.awt.*;
import java.awt.event.MouseEvent;

import javax.swing.*;

import april.util.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import april.vis.*;
import april.image.*;

import finallab.lcmtypes.*;

public class Scoreboard
{
	int human = 0;
	int robot = 0;

	JFrame jf;
	VisWorld vw;
	VisLayer vl;
	VisCanvas vc;
	boolean self;

	public Scoreboard(boolean _self, JFrame _jf, VisWorld _vw, VisLayer _vl, VisCanvas _vc)
	{
		if(_self)
		{
			jf = new JFrame("Scoreboard");
			vw = new VisWorld();
			vl = new VisLayer(vw);
			vc = new VisCanvas(vl);
			jf.setLayout(new BorderLayout());
			jf.add(vc, BorderLayout.CENTER);
			jf.setSize(800,600);
			jf.setVisible(true);
			jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			vl.cameraManager.uiLookAt(new double[] {-2.66075, 1.22066, 1.70393 },
				new double[] {1.75367, -0.06226,  0.00000 },
				new double[] {0.33377, -0.09695,  0.93766 }, true);
		}
		else
		{
			jf = _jf;
			vw = _vw;
			vl = _vl;
			vc = _vc;
		}
		drawScoreboard();
	}

	public void drawScoreboard()
	{
		//do vis stuff here with human and robot scores
		VisWorld.Buffer vb = vw.getBuffer("Scoreboard");		
		VisChain background = new VisChain(LinAlg.translate(0,0,-.05),new VzBox(8,4,0.05,new VzMesh.Style(Color.gray)));
		VzBox horizLines = new VzBox(8,.1,.1, new VzMesh.Style(Color.black));
		VzBox vertLines = new VzBox(.1,4,.1, new VzMesh.Style(Color.black));
		VisChain boarder = new VisChain(vertLines,LinAlg.translate(0,2,0),horizLines,LinAlg.translate(0,-4,0),horizLines,LinAlg.translate(0,3.5,0),horizLines,LinAlg.translate(-4,-1.5,0),vertLines,LinAlg.translate(8,0,0),vertLines);
		VzText robotName = new VzText(VzText.ANCHOR.CENTER,"<<sansserif-bold-24,black>>" + "Robot");
		VzText humanName = new VzText(VzText.ANCHOR.CENTER,"<<sansserif-bold-24,black>>" + "Human");
		VisChain rNames = new VisChain(LinAlg.translate(2,1.75,0),LinAlg.scale(.01,.01,1),robotName);
		VisChain hNames = new VisChain(LinAlg.translate(-2,1.75,0),LinAlg.scale(.01,.01,1),humanName);

		VzText robotScore = new VzText(VzText.ANCHOR.CENTER,"<<sansserif-bold-24,red>>" + robot);
		VzText humanScore = new VzText(VzText.ANCHOR.CENTER,"<<sansserif-bold-24,blue>>" + human);
		VisChain rScore = new VisChain(LinAlg.translate(2,0,0),LinAlg.scale(.1,.1,1),robotScore);
		VisChain hScore = new VisChain(LinAlg.translate(-2,0,0),LinAlg.scale(.1,.1,1),humanScore);
		VisChain scoreBoardArt = new VisChain(LinAlg.scale(.5,.5,.5),LinAlg.translate(0,10,2),LinAlg.rotateX(Math.PI/2),background,boarder,rNames,hNames,rScore,hScore);


		vb.addBack(scoreBoardArt);
		// vb.addBack(boarder);
		// vb.addBack(rNames);
		// vb.addBack(hNames);
		// vb.addBack(rScore);
		// vb.addBack(hScore);

		vb.swap();
	}

	public void clearScoreboard()
	{
		human = 0;
		robot = 0;
		drawScoreboard();
	}

	public void addToRobot()
	{
		robot++;
		drawScoreboard();
	}

	public void addToHuman()
	{
		human++;
		drawScoreboard();
	}

	static public void main(String[] args)
	{
		Scoreboard sb = new Scoreboard(true, null, null, null, null);
		BufferedReader in;
		PrintStream out;
		String my_string;

		in = new BufferedReader(new InputStreamReader(System.in));
		out = System.out;

		while(true)
		{
			try
			{
				my_string = in.readLine();
				out.println(my_string);
			}
			catch(Exception e)
			{
				my_string = "c";
			}
			if(my_string.equals("r"))
			{
				sb.addToRobot();
			}
			else if(my_string.equals("h"))
			{
				sb.addToHuman();
			}
			else if(my_string.equals("c"))
			{
				sb.clearScoreboard();
			}
		}
	}
}