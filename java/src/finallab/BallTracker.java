package finallab;

import java.io.*;
import java.util.*;
import java.awt.*;

import april.util.*;
import april.jmat.*;
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

	public ArrayList<Statistics> analyze()
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
		HashMap <Integer, Statistics> map = new HashMap<Integer, Statistics>();
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				int access = y*width+x;
				if(!thresholdMap[access])
					continue;
				if(finder.find(access) == access)
				{
					Statistics input = new Statistics();
					input.update(x,y);
					map.put(access, input);
				}
				else if(map.containsKey(finder.find(access)))
				{
					Statistics output = map.get(finder.find(access));
					output.update(x,y);
					map.put(finder.find(access),output);
				}
			}
		}
		Iterator obIter = map.keySet().iterator();
		ArrayList<Statistics> blobs = new ArrayList<Statistics> ();
		while(obIter.hasNext())
		{
			Integer key = (Integer) obIter.next();
			Statistics value = (Statistics) map.get(key);
			if(value.N > 400)
			{
				blobs.add(value);
			}
		}
		return blobs;
	}
}
