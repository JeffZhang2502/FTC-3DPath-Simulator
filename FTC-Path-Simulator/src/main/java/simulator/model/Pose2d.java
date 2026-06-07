package simulator.model;

/**
 * Represents a 2D pose with position and heading.
 * Coordinate system follows Road Runner convention:
 * the field center is (0, 0).
 */
public class Pose2d {
    public double x;
    public double y;
    public double heading;

    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.3f, y=%.3f, heading=%.3f)", x, y, heading);
    }
}
