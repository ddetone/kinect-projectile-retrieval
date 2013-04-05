package finallab;

import java.util.*;
import java.io.*;

import april.util.*;
import april.jmat.*;

public class CatchController
{
	Projectile predictor;
	BallTracker finder;
	KinectView viewer;
	boolean display = false;

	CatchController(boolean _display)
	{
		predictor = new Projectile();
		//finder = new BallTracker();
		display = _display;
		viewer = new KinectView(_display);

	}


	public static void main(String[] args)
	{
	}
		
}

	