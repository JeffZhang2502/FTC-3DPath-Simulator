package simulator.model;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents an elastic ball with full 3D projectile physics.
 *
 * <p>The ball experiences gravity (g = 386.09 in/s²), bounces off
 * the floor and field walls with a configurable coefficient of
 * restitution, and logs its trajectory for 3D rendering.</p>
 *
 * <p>Collision detection checks all four perimeter walls and the
 * floor plane.  Each bounce dissipates energy according to the
 * restitution coefficient and applies surface friction to
 * horizontal velocity components.</p>
 */
public class Ball {

    /** Gravitational acceleration in inches per second². */
    public static final double G = 386.09;

    /** Default ball radius in inches (≈ tennis ball size). */
    public static final double DEFAULT_RADIUS = 1.5;

    /** Default coefficient of restitution (0 = perfectly inelastic,
     *  1 = perfectly elastic).  FTC balls are moderately bouncy. */
    public static final double DEFAULT_RESTITUTION = 0.72;

    /** Minimum velocity threshold — ball sleeps below this speed. */
    private static final double SLEEP_SPEED = 3.0;

    /** Floor friction coefficient applied on bounce. */
    private static final double FLOOR_FRICTION = 0.75;

    // ---- position (world inches) ----

    private double x, y, z;

    // ---- velocity components (inches/sec) ----

    private double vx, vy, vz;

    // ---- launch origin (for analytic recomputation) ----

    private double x0, y0, z0;

    // ---- physical properties ----

    private double radius;
    private double restitution;

    // ---- state ----

    private double flightTime;       // seconds since last launch / bounce reset
    private boolean active;

    // ---- trajectory log ----

    private final List<double[]> trajectory = new ArrayList<>();

    // ---- constructors ----

    public Ball() {
        this(DEFAULT_RADIUS, DEFAULT_RESTITUTION);
    }

    public Ball(double radius, double restitution) {
        this.radius = radius;
        this.restitution = Math.max(0.0, Math.min(1.0, restitution));
        reset();
    }

    // ---- lifecycle ----

    /** Reset ball to idle state, clearing trajectory. */
    public void reset() {
        x = y = z = 0;
        vx = vy = vz = 0;
        x0 = y0 = z0 = 0;
        flightTime = 0;
        active = false;
        trajectory.clear();
    }

    /**
     * Place the ball at a static world position without launching it.
     * The ball is visible but not in flight.
     */
    public void placeAt(double x, double y, double z) {
        reset();
        this.x = x;
        this.y = y;
        this.z = z;
        this.x0 = x;
        this.y0 = y;
        this.z0 = z;
        record();
    }

    /**
     * Launch the ball from a world position with given initial conditions.
     *
     * @param x0              world X (inches)
     * @param y0              world Y (inches)
     * @param z0              launch height above floor (inches)
     * @param headingRad      launch azimuth (robot heading)
     * @param launchAngleDeg  elevation above horizontal (degrees)
     * @param v0              muzzle velocity magnitude (inches/sec)
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

    // ---- physics update ----

    /**
     * Advance ball physics by {@code dt} seconds.
     *
     * <p>Uses analytic projectile equations recomputed from absolute
     * time to avoid integration drift.  After each bounce the origin
     * is reset so the parabola stays exact.</p>
     */
    public void update(double dt) {
        if (!active) return;

        flightTime += dt;

        // Analytic projectile from current origin.
        double t = flightTime;
        x = x0 + vx * t;
        y = y0 + vy * t;
        z = z0 + vz * t - 0.5 * G * t * t;

        // ---- floor collision ----
        if (z <= radius) {
            z = radius;
            if (vz < 0) {
                // Elastic bounce with friction.
                vz = -vz * restitution;
                vx *= FLOOR_FRICTION;
                vy *= FLOOR_FRICTION;
                resetOrigin();

                if (Math.abs(vz) < SLEEP_SPEED) {
                    vz = 0;
                    if (Math.hypot(vx, vy) < SLEEP_SPEED) {
                        active = false;
                    }
                }
            } else {
                vz = 0;
            }
        }

        record();
    }

    /**
     * Check and resolve collisions with the four perimeter walls.
     *
     * <p>Each wall collision reflects the velocity component normal
     * to the wall and applies the restitution coefficient.</p>
     *
     * @param halfField half the field side length (inches)
     * @return true if any wall collision occurred this frame
     */
    public boolean checkWallCollision(double halfField) {
        if (!active) return false;
        boolean hit = false;

        // North wall  (world +Y)
        if (y + radius > halfField) {
            y = halfField - radius;
            if (vy > 0) { vy = -vy * restitution; hit = true; }
        }
        // South wall  (world -Y)
        if (y - radius < -halfField) {
            y = -halfField + radius;
            if (vy < 0) { vy = -vy * restitution; hit = true; }
        }
        // East wall   (world +X)
        if (x + radius > halfField) {
            x = halfField - radius;
            if (vx > 0) { vx = -vx * restitution; hit = true; }
        }
        // West wall   (world -X)
        if (x - radius < -halfField) {
            x = -halfField + radius;
            if (vx < 0) { vx = -vx * restitution; hit = true; }
        }

        if (hit) {
            resetOrigin();
        }
        return hit;
    }

    /**
     * Bounce the ball off an arbitrary surface given its unit normal.
     *
     * <p>Uses the standard reflection formula:
     *   v' = v − (1 + e) · (v · n) · n
     * where e is the coefficient of restitution.</p>
     *
     * @param nx  surface normal X component
     * @param ny  surface normal Y component
     * @param nz  surface normal Z component
     */
    public void bounce(double nx, double ny, double nz) {
        double vDotN = vx * nx + vy * ny + vz * nz;
        if (vDotN < 0) {   // ball moving toward surface
            double impulse = (1.0 + restitution) * (-vDotN);
            vx += impulse * nx;
            vy += impulse * ny;
            vz += impulse * nz;
            resetOrigin();
        }
    }

    // ---- internal helpers ----

    /** Reset the analytic origin to the current position so the
     *  parabolic arc restarts from here. */
    private void resetOrigin() {
        x0 = x; y0 = y; z0 = z;
        flightTime = 0;
    }

    private void record() {
        trajectory.add(new double[] {x, y, z});
    }

    // ---- direct recompute (for rendering at arbitrary time) ----

    /**
     * Recompute ball position at a given absolute time without
     * advancing the simulation.  Used by the renderer to draw
     * predicted trajectory arcs.
     */
    public void recomputeAt(double t, double x0, double y0, double z0,
                            double headingRad, double launchAngleDeg, double v0) {
        this.flightTime = t;
        double theta = Math.toRadians(launchAngleDeg);
        double vh = v0 * Math.cos(theta);
        this.x = x0 + vh * Math.cos(headingRad) * t;
        this.y = y0 + vh * Math.sin(headingRad) * t;
        this.z = z0 + v0 * Math.sin(theta) * t - 0.5 * G * t * t;
    }

    // ---- queries ----

    public double getX()            { return x; }
    public double getY()            { return y; }
    public double getZ()            { return z; }
    public double getVx()           { return vx; }
    public double getVy()           { return vy; }
    public double getVz()           { return vz; }
    public double getSpeed()        { return Math.sqrt(vx*vx + vy*vy + vz*vz); }
    public double getRadius()       { return radius; }
    public double getRestitution()  { return restitution; }
    public double getFlightTime()   { return flightTime; }
    public boolean isActive()       { return active; }

    public void setActive(boolean active) { this.active = active; }

    /** Returns a copy of the (x, y, z) trajectory for rendering. */
    public List<double[]> getTrajectory() {
        return new ArrayList<>(trajectory);
    }

    @Override
    public String toString() {
        return String.format("Ball[pos=(%.1f,%.1f,%.1f) v=(%.0f,%.0f,%.0f) r=%.1f e=%.2f %s]",
                x, y, z, vx, vy, vz, radius, restitution, active ? "●" : "○");
    }
}
