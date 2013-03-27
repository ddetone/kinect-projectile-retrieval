package finallab;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import org.openkinect.freenect.*;
import org.openkinect.freenect.util.*;


import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;
import april.jmat.geom.*;

import finallab.lcmtypes.*;

public class BallTracker
{
	boolean[] thresholdMap;
	int width;
	int height;
	int size;
	UnionFind finder;

	public BallTracker(boolean[] _thresholdMap, int _width, int _height)
	{
		thresholdMap = _thresholdMap;
		width = _width;
		height = _height;
		size = width*height;
		finder = new UnionFind(size);
	}

	public void analyze()
	{
		for(int y = 0; y < height-1; y++)
			for(int x = 0; x < width-1; x++)
			{
				int access = y*width+x;
				int plusX = y*width+x+1;
				int plusY = (y+1)*width+x;
				if(thresholdMap[access])
				{
					if(thresholdMap[plusX])
						finder.join(access,plusX);
					if(thresholdMap[plusY])
						finder.join(access,plusY);
				}
			}
	}
}