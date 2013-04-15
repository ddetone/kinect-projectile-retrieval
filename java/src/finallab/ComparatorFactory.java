package finallab;

import java.util.*;
import java.awt.*;

public class ComparatorFactory {
	
	//sorts statistics object by pixel size decreasing
	public static class StatisticsCompareSize implements Comparator<Statistics> {
		@Override
		public int compare(Statistics s1, Statistics s2) {
			if (s1.N < s2.N) {
				return 1;
			}
			else if (s1.N > s2.N) {
				return -1;
			}
			else {
				return 0;
			}
		}
	}
	//sorts statistics object by y pixel, lowest in frame to highest in frame
	public static class StatisticsCompareYPix implements Comparator<Statistics> {
		@Override
		public int compare(Statistics s1, Statistics s2) {
			if (s1.max_y < s2.max_y) {
				return 1;
			}
			else if (s1.max_y > s2.max_y) {
				return -1;
			}
			else {
				return 0;
			}
		}
	}

	public static StatisticsCompareSize getStatisticsCompareSize() {
		return new StatisticsCompareSize();
	}
	public static StatisticsCompareYPix getStatisticsCompareYPix() {
		return new StatisticsCompareYPix();
	}
}