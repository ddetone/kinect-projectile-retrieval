package finallab;

public class Point3D {
	double x;
	double y;
	double z;

	public Point3D() {
		x = 0d;
		y = 0d;
		z = 0d;
	}
	public Point3D(double _x, double _y, double _z) {
		x = _x;
		y = _y;
		z = _z;
	}
	public String toString() {
		return "Point3D(" + x + ", " + y + ", " + z + ")";
	}
	public Point3D clone() {
		return new Point3D(x, y, z);
	}
}