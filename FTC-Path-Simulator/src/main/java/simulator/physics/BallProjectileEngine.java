package simulator.physics;

import java.util.List;
import simulator.model.Ball;

/**
 * 3D projectile-motion engine for a ball launched by the robot.
 *
 * <p>Simulates parabolic flight under gravity (g = 386.09 in/s²)
 * using the elastic {@link Ball} model.  Evaluates whether the ball
 * lands inside a scoring goal and handles rim-bounce physics for
 * near-miss deflections.</p>
 *
 * <p>The engine delegates core physics (gravity, wall/floor bounce)
 * to {@link Ball} and adds goal-specific scoring logic on top.</p>
 */
public class BallProjectileEngine {

    /** Gravitational acceleration in inches per second². */
    public static final double G = Ball.G;

    /** Goal rim height in inches above the field floor. */
    public static final double GOAL_HEIGHT = 38.75;

    /** Goal acceptance radius in inches. */
    public static final double GOAL_RADIUS = 8.0;

    /** Coefficient of restitution for rim bounce (ball vs metal). */
    public static final double RESTITUTION = 0.58;

    // ---- ball ----

    private final Ball ball;
    private boolean scored;

    // ---- World position & radius of a scoring goal. ----

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

    public BallProjectileEngine() {
        this.ball = new Ball(Ball.DEFAULT_RADIUS, Ball.DEFAULT_RESTITUTION);
    }

    public BallProjectileEngine(Ball ball) {
        this.ball = ball;
    }

    public void reset() {
        ball.reset();
        scored = false;
    }

    /**
     * Launch the ball from the robot's current pose.
     *
     * @param x0             robot world X (inches)
     * @param y0             robot world Y (inches)
     * @param z0             launch height above floor (inches)
     * @param headingRad     robot heading in radians
     * @param launchAngleDeg launch elevation above horizontal (degrees)
     * @param v0             muzzle velocity magnitude (inches/sec)
     */
    public void launch(double x0, double y0, double z0,
                       double headingRad, double launchAngleDeg, double v0) {
        reset();
        ball.launch(x0, y0, z0, headingRad, launchAngleDeg, v0);
    }

    /** Place the ball at a static position (visible but idle). */
    public void placeAt(double x, double y, double z) {
        reset();
        ball.placeAt(x, y, z);
    }

    /**
     * Independent physics-only update — gravity, floor bounce, wall
     * bounce — without any goal-scoring check.  Call this every frame
     * so the ball exists as a free, independent entity on the field.
     *
     * @param dt        time delta (seconds)
     * @param halfField half the field side length for wall collision
     */
    public void updatePhysics(double dt, double halfField) {
        if (!ball.isActive()) return;
        ball.update(dt);
        ball.checkWallCollision(halfField);
    }

    /**
     * Advance the simulation by {@code dt} seconds.
     *
     * @param dt        time delta (seconds)
     * @param halfField half the field side length for wall collision
     * @param goal      the target goal box to check against (null = physics only)
     * @return status message, or null if still in flight
     */
    public String update(double dt, double halfField, GoalBox goal) {
        if (!ball.isActive()) return null;

        // Delegate core physics to Ball.
        ball.update(dt);

        // Check wall collisions (elastic bounce off perimeter).
        ball.checkWallCollision(halfField);

        if (!ball.isActive()) return null;

        // If no goal to check against, physics-only update.
        if (goal == null) return null;

        double x = ball.getX();
        double y = ball.getY();
        double z = ball.getZ();

        // ---- goal-plane scoring & rim-bounce (§10 scoring spec) ----
        if (z <= goal.height) {
            double dx = x - goal.cx;
            double dy = y - goal.cy;
            double dist = Math.hypot(dx, dy);

            // Inside the inner-radius → SCORE.
            if (dist <= goal.radius) {
                ball.setActive(false);
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

                // Reflect velocity using Ball's bounce method.
                double vx = ball.getVx();
                double vy = ball.getVy();
                double vDotN = vx * nx + vy * ny;
                if (vDotN < 0) {   // ball moving toward rim
                    ball.bounce(nx, ny, 0.0);

                    return String.format("💥 Rim bounce! V=(%.0f,%.0f,%.0f) in/s, "
                                         + "ball deflected.",
                                         ball.getVx(), ball.getVy(), ball.getVz());
                }
            }

            // Ball missed the goal entirely.
            ball.setActive(false);
            return String.format("🌍 Ball missed goal at (%.1f, %.1f), dist=%.1f in.",
                                 x, y, dist);
        }

        // Ball hit the floor without reaching goal height.
        if (!ball.isActive()) {
            return String.format("🌍 Ball grounded at (%.1f, %.1f).", x, y);
        }

        return null;   // still in flight
    }

    /** Directly recompute position for given absolute time (used for rendering). */
    public void recomputeAt(double t, double x0, double y0, double z0,
                            double headingRad, double launchAngleDeg, double v0) {
        ball.recomputeAt(t, x0, y0, z0, headingRad, launchAngleDeg, v0);
    }

    // ---- queries ----

    /** Returns the underlying elastic Ball model. */
    public Ball getBall() {
        return ball;
    }

    public double getX()              { return ball.getX(); }
    public double getY()              { return ball.getY(); }
    public double getZ()              { return ball.getZ(); }
    public double getVx()             { return ball.getVx(); }
    public double getVy()             { return ball.getVy(); }
    public double getVz()             { return ball.getVz(); }
    public boolean isActive()         { return ball.isActive(); }
    public boolean isScored()         { return scored; }
    public double getFlightTime()     { return ball.getFlightTime(); }

    /** Returns a copy of the (x,y,z) trajectory points for rendering. */
    public List<double[]> getTrajectory() {
        return ball.getTrajectory();
    }
}
