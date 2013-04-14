package finallab;

import java.lang.Math;
import java.awt.*;

public class Statistics
{
    private static final int MAX_VALUE = 2147483647;
    private static final double DMAX_VALUE = 2147483647.0;
    public double Mxx, Mxy, Myy;
    public double Mx, My;
    public int Mz;
    public int closestDepth;
    public Point closestPixel;
    int N;
    public int max_x, max_y, min_x, min_y;
    public int center_x, center_y;
    public int ColorID;

    public Statistics()
    {
        Mxx = 0; Mxy = 0; Myy = 0;
        Mx = 0; My = 0; Mz = 0;
        N = 0;
        max_x = 0;
        max_y = 0;
        min_x = MAX_VALUE;
        min_y = MAX_VALUE;
        center_y = 0;
        center_x = 0;
        closestDepth = 2047;
    }

    public void update(int pixelX, int pixelY)
    {
        Mxx += (pixelX * pixelX);
        Mxy += (pixelX * pixelY);
        Myy += (pixelY * pixelY);
        Mx += pixelX;
        My += pixelY;


        N++;

        if(pixelX < min_x)
            min_x = pixelX;

        if(pixelY < min_y)
            min_y = pixelY;

        if(pixelX > max_x)
            max_x = pixelX;

        if(pixelY > max_y)
            max_y = pixelY;
    }

    public void update(int pixelX, int pixelY, int depth)
    {
        Mxx += (pixelX * pixelX);
        Mxy += (pixelX * pixelY);
        Myy += (pixelY * pixelY);
        Mx += pixelX;
        My += pixelY;
        Mz += depth;

        N++;

        if(pixelX < min_x)
            min_x = pixelX;

        if(pixelY < min_y)
            min_y = pixelY;

        if(pixelX > max_x)
            max_x = pixelX;

        if(pixelY > max_y)
            max_y = pixelY;

        if(depth < closestDepth)
        {
            closestDepth = depth;
            closestPixel = new Point(pixelX,pixelY);
        }
    }

    public double Ux()
    {
        return (Mx/(double) N);
    }

    public double Uy()
    {
        return (My/(double) N);
    }

    public int Uz() {
        return (Mz/N);
    }

    public double Cxx()
    {
        if(N == 0)
            return DMAX_VALUE;
        double xAvg = this.Ux();
        double corr = (Mxx/(double)N);
        return (corr - (xAvg*xAvg));
    }

    public double Cxy()
    {
        if(N == 0)
            return DMAX_VALUE;
        double xAvg = this.Ux();
        double yAvg = this.Uy();
        double corr = (Mxy/(double)N);
        return (corr - (xAvg*yAvg));
    }

    public double Cyy()
    {
        if(N == 0)
            return DMAX_VALUE;
        double yAvg = this.Uy();
        double corr = (Myy/(double)N);
        return (corr - (yAvg*yAvg));
    }

    public double abs()
    {
        if(N == 0)
            return DMAX_VALUE;

        double value = Math.abs(this.Cyy()-this.Cxx());
        return value;
    }

    public Point center()
    {
        center_x = (min_x + max_x)/2;
        center_y = (min_y + max_y)/2;
        return new Point(center_x, center_y);
    }
}
