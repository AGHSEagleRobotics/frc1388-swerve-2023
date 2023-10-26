package frc.robot.helpers;

public class Point3d {
    public double x;
    public double y;
    public double z;

    public Point3d (double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public String toString() {
        return "x: " + x + "\ny: " + y + "\nz: " + z;
    }
}
