package simulator.physics;

import java.util.ArrayList;
import java.util.List;

/**
 * 3D projectile-motion engine for a ball launched by the robot.
 *
 * <p>Simulates parabolic flight under gravity (g = 386.09 in/s²).
 * Tracks the ball's (x, y, z) position over time and evaluates
 * whether it lands inside a scoring goal.</p>
 */
public class BallProjectileEngine {

    /** Gravitational acceleration in inches per second². */
    public static final double G = 386.09;

    /** Goal rim height in inches above the field floor. */
    public static final double GOAL_HEIGHT = 38.75;

    /** Goal acceptance radius in inches. */
    public static final double GOAL_RADIUS = 8.0;

    /** Coefficient of restitution for rim bounce (ball vs metal). */
    public static final double RESTITUTION = 0.58;

    // ---- ball state ----

    private double x0, y0, z0;   // launch origin
    private double x, y, z;      // current world position (inches)
    private double vx, vy, vz;   // velocity components (inches/sec)
    private double flightTime;   // seconds since launch
    private boolean active;
    private boolean scored;

    // ---- trajectory log ----

    private final List<double[]> trajectory = new ArrayList<>();

    /** World position & radius of a scoring goal. */
    public static class GoalBox {
        public final double cx, cy;   // center on the field floor
        public final double radius;   // acceptance radius
        public final double height;   // rim height

        public GoalBox(double cx, double cy, double radius, double height) {
            this.cx = cx; this.cy = cy;
            this.radius = radius;
            this.height = height;
        }
    }

    // ---- lifecycle ----

    public void reset() {
        x = y = z = 0;
        vx = vy = vz = 0;
        flightTime = 0;
        active = false;
        scored = false;
        trajectory.clear();
    }

    /**
     * Launch the ball from the robot's current pose.
     *
     * @param x0         robot world X (inches)
     * @param y0         robot world Y (inches)
     * @param z0         launch height above floor (inches), typically ~8"
     * @param headingRad robot heading in radians
     * @param launchAngleDeg launch elevation above horizontal (degrees)
     * @param v0         muzzle velocity magnitude (inches/sec)
     */
    public void launch(double x0, double y0, double z0,
                       double headingRad, double launchAngleDeg, double v0) {
        reset();
        this.x0 = x0;
        this.y0 = y0;
        this.z0 = z0;

        double theta = Math.toRadians(launchAngleDeg);
        double cosTheta = Math.cos(theta);

        this.vx = v0 * cosTheta * Math.cos(headingRad);
        this.vy = v0 * cosTheta * Math.sin(headingRad);
        this.vz = v0 * Math.sin(theta);

        this.active = true;
        this.x = x0; this.y = y0; this.z = z0;
        record();
    }

    /**
     * Advance the simulation by {@code dt} seconds.
     *
     * @param goal  the target goal box to check against
     * @return status message, or null if still in flight
     */
    public String update(double dt, GoalBox goal) {
        if (!active) return null;

        flightTime += dt;

        // Recompute from absolute time — no accumulation drift.
        double t = flightTime;
        x = x0 + vx * t;
        y = y0 + vy * t;
        z = z0 + vz * t - 0.5 * G * t * t;

        record();

        // ---- goal-plane scoring & rim-bounce (§10 scoring spec) ----
        if (z <= goal.height) {
            double dx = x - goal.cx;
            double dy = y - goal.cy;
            double dist = Math.hypot(dx, dy);

            // Inside the inner-radius → SCORE.
            if (dist <= goal.radius) {
                active = false;
                scored = true;
                return String.format("⚽ GOAL! Ball inside rim at (%.1f, %.1f), "
                                     + "dist=%.1f in ≤ R=%.1f in",
                                     x, y, dist, goal.radius);
            }

            // Near-miss: ball clipped the rim → elastic bounce.
            double rimBand = goal.radius + 6.0;   // rim tolerance band
            if (dist <= rimBand) {
                // Compute contact normal from goal centre to ball.
                double nx = dx / dist;
                double ny = dy / dist;
                double nz = 0.0;   // rim is horizontal

                // Reflect velocity:  v' = v − (1+e)(v·n)n
                double vDotN = vx * nx + vy * ny + vz * nz;
                if (vDotN < 0) {   // ball moving toward rim
                    double bounceFactor = (1.0 + RESTITUTION) * (-vDotN);
                    vx += bounceFactor * nx;
                    vy += bounceFactor * ny;
                    vz += bounceFactor * nz;

                    // Recompute from new vx/vy/vz from the bounce point.
                    x0 = x; y0 = y; z0 = z;
                    flightTime = 0;
                    active = true;   // continue flight with new initial conditions

                    return String.format("💥 Rim bounce! V=(%.0f,%.0f,%.0f) in/s, "
                                         + "ball deflected.", vx, vy, vz);
                }
            }

            // Ball missed the goal entirely — fall through to floor check.
            active = false;
            z = 0;
            return String.format("🌍 Ball missed goal at (%.1f, %.1f), dist=%.1f in.",
                                 x, y, dist);
        }

        // Ball hit the floor without reaching goal height.
        if (z <= 0) {
            active = false;
            z = 0;
            return String.format("🌍 Ball grounded at (%.1f, %.1f) — too low.", x, y);
        }

        return null;   // still in flight
    }

    /** Directly recompute position for given absolute time (used for rendering). */
    public void recomputeAt(double t, double x0, double y0, double z0,
                            double headingRad, double launchAngleDeg, double v0) {
        this.flightTime = t;
        double theta = Math.toRadians(launchAngleDeg);
        double vh = v0 * Math.cos(theta);
        this.x = x0 + vh * Math.cos(headingRad) * t;
        this.y = y0 + vh * Math.sin(headingRad) * t;
        this.z = z0 + v0 * Math.sin(theta) * t - 0.5 * G * t * t;
    }

    private void record() {
        trajectory.add(new double[] {x, y, z});
    }

    // ---- queries ----

    public double getX()       { return x; }
    public double getY()       { return y; }
    public double getZ()       { return z; }
    public boolean isActive()  { return active; }
    public boolean isScored()  { return scored; }
    public double getFlightTime() { return flightTime; }

    /** Returns a copy of the (x,y,z) trajectory points for rendering. */
    public List<double[]> getTrajectory() {
        return new ArrayList<>(trajectory);
    }
}
