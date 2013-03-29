package finallab;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import april.util.*;
import april.jmat.*;
import april.jmat.geom.*;

import java.awt.*;
import java.awt.image.*;

import finallab.lcmtypes.*;

public class BallTracker
{
	int width;
	int height;
	int size;
	UnionFind finder;

	BufferedImage output;
	JFrame outputFrame;
	JImage outputImage;

	public BallTracker(int _width, int _height)
	{
		width = _width;
		height = _height;
		size = width*height;

		output = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
		outputFrame = new JFrame("KinectView");
		outputImage = new JImage();
		outputFrame.setLayout(new GridLayout(1,1));
		outputFrame.add(outputImage, 0, 0);
		outputFrame.setSize(640,480);
		outputFrame.setVisible(true);
		outputFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public ArrayList<Statistics> analyze2(boolean[] thresholdMap)
	{
		finder = new UnionFind(size);
		HashMap <Integer, Statistics> map = new HashMap<Integer, Statistics>();
		int y = 0;
		for(int x = 0; x < width; x++)
		{
			int access = x;
			int plusX = x+1;
			int plusY = width+x;
			if(thresholdMap[access])
			{
				output.setRGB(x,y,0xFFFF0000);
				if((x != width-1) && thresholdMap[plusX])
					finder.join(access,plusX);
				if(thresholdMap[plusY])
					finder.join(access,plusY);
			}
			else
			{
				output.setRGB(x,y,0xFFFFFFFF);
			}
		}
		for(y = 1; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				int delayedPointer = (y-1)*width+x;
				int access = y*width+x;
				int plusX = y*width+x+1;
				int plusY = (y+1)*width+x;
				if(thresholdMap[access])
				{
					output.setRGB(x,y,0xFFFF0000);
					if((x != width-1) && thresholdMap[plusX])
						finder.join(access,plusX);
					if((y != height-1) && thresholdMap[plusY])
						finder.join(access,plusY);
				}
				else
				{
					output.setRGB(x,y,0xFFFFFFFF);
				}
				if(!thresholdMap[delayedPointer])
					continue;
				if(finder.find(delayedPointer) == delayedPointer)
				{
					Statistics input = new Statistics();
					input.update(x,y);
					map.put(delayedPointer, input);
				}
				else if(map.containsKey(finder.find(delayedPointer)))
				{
					Statistics output = map.get(finder.find(delayedPointer));
					output.update(x,y);
					map.put(finder.find(delayedPointer),output);
				}
			}
		}
		y = height-1;
		for(int x = 0; x < width; x++)
		{
			int delayedPointer = y*width+x;
			if(!thresholdMap[delayedPointer])
				continue;
			if(finder.find(delayedPointer) == delayedPointer)
			{
				Statistics input = new Statistics();
				input.update(x,y);
				map.put(delayedPointer, input);
			}
			else if(map.containsKey(finder.find(delayedPointer)))
			{
				Statistics output = map.get(finder.find(delayedPointer));
				output.update(x,y);
				map.put(finder.find(delayedPointer),output);
			}
		}
		Iterator obIter = map.keySet().iterator();
		ArrayList<Statistics> blobs = new ArrayList<Statistics> ();
		while(obIter.hasNext())
		{
			Integer key = (Integer) obIter.next();
			Statistics value = (Statistics) map.get(key);
			blobs.add(value);
		}
		outputImage.setImage(output);
		return blobs;
	}

	public ArrayList<Statistics> analyze(boolean[] thresholdMap)
	{
		finder = new UnionFind(size);
		HashMap <Integer, Statistics> map = new HashMap<Integer, Statistics>();
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				int access = y*width+x;
				int plusX = y*width+x+1;
				int plusY = (y+1)*width+x;
				if(thresholdMap[access])
				{
					output.setRGB(x,y,0xFFFF0000);
					if((x != width-1) && thresholdMap[plusX])
						finder.join(access,plusX);
					if((y != height-1) && thresholdMap[plusY])
						finder.join(access,plusY);
				}
				else
				{
					output.setRGB(x,y,0xFFFFFFFF);
				}
			}
		}
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
			blobs.add(value);
		}
		outputImage.setImage(output);
		return blobs;
	}
	public ArrayList<Statistics> analyzeDepth(ByteBuffer buf) {
		finder = new UnionFind(size);
		HashMap <Integer, Statistics> map = new HashMap<Integer, Statistics>();
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				int access = y*width+x;
				int plusX = y*width+x+1;
				int plusY = (y+1)*width+x;

				if((x != width-1)) {
					if (Math.abs(getDepth(buf, access) - getDepth(buf, plusX)) < 6) {
						finder.join(access,plusX);
						output.setRGB(x + 1,y,0xFFFF0000);
					}
					else {
						output.setRGB(x + 1,y,0x00000000);
					}
				}
				if((y != height-1)) {
					if (Math.abs(getDepth(buf, access) - getDepth(buf, plusX)) < 6) {
						finder.join(access,plusY);
						output.setRGB(x,y + 1,0xFFFF0000);
					}
					else {
						output.setRGB(x,y+1,0x00000000);
					}
					
				}
			}

		}
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				int access = y*width+x;
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
			blobs.add(value);
		}
		outputImage.setImage(output);
		return blobs;
	
	}
	public int getDepth(ByteBuffer bb, int index) {
		int depth = 0;
		byte byte1 = bb.get(index * 2);
		byte byte2 = bb.get(index * 2 + 1);
		depth = byte2 & 0x3;
		depth = depth << 8;
		depth = depth | (byte1 & 0xFF);
		return depth & 0x3FF;
	}
}
