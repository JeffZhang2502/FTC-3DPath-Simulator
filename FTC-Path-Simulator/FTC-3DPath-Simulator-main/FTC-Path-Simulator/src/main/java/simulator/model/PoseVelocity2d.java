package simulator.model;

/**
 * Represents the velocity of a robot in 2D space.
 * vx and vy are in the robot's local frame (body-centric).
 */
public class PoseVelocity2d {
    public double vx;
    public double vy;
    public double vomega;

    public PoseVelocity2d() {
        this(0.0, 0.0, 0.0);
    }

    public PoseVelocity2d(double vx, double vy, double vomega) {
        this.vx = vx;
        this.vy = vy;
        this.vomega = vomega;
    }

    @Override
    public String toString() {
        return String.format("PoseVelocity2d(vx=%.3f, vy=%.3f, vomega=%.3f)", vx, vy, vomega);
    }
}
