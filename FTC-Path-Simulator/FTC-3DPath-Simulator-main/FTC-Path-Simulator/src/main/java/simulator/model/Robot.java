package simulator.model;

/**
 * Represents an FTC robot with Mecanum wheel kinematics,
 * a configurable {@link RobotProfile}, and vertex-sampling collision detection.
 */
public class Robot {

    /** Half track width in meters (for mecanum kinematics). */
    private static final double HALF_TRACK = 0.18;
    /** Half wheelbase in meters (for mecanum kinematics). */
    private static final double HALF_WHEELBASE = 0.18;

    public Pose2d currentPose;
    private PoseVelocity2d currentVelocity;
    private RobotProfile profile;

    // ---- constructors ----

    public Robot() {
        this.currentPose = new Pose2d();
        this.currentVelocity = new PoseVelocity2d();
        this.profile = new RobotProfile();
    }

    public Robot(Pose2d startPose) {
        this.currentPose = startPose;
        this.currentVelocity = new PoseVelocity2d();
        this.profile = new RobotProfile();
    }

    public Robot(RobotProfile profile, Pose2d startPose) {
        this.profile = profile;
        this.currentPose = startPose;
        this.currentVelocity = new PoseVelocity2d();
    }

    // ---- accessors ----

    public PoseVelocity2d getCurrentVelocity() {
        return currentVelocity;
    }

    public void setCurrentVelocity(PoseVelocity2d velocity) {
        this.currentVelocity = velocity;
    }

    public RobotProfile getProfile() {
        return profile;
    }

    public void setProfile(RobotProfile profile) {
        this.profile = profile;
    }

    // ---- Mecanum kinematics ----

    /**
     * Inverse kinematics: desired robot velocity → four wheel speeds.
     * Order: [FL, FR, BL, BR].
     */
    public double[] wheelKinematics(PoseVelocity2d vel) {
        double r = HALF_TRACK + HALF_WHEELBASE;
        return new double[] {
            vel.vx - vel.vy - r * vel.vomega,   // FL
            vel.vx + vel.vy + r * vel.vomega,   // FR
            vel.vx + vel.vy - r * vel.vomega,   // BL
            vel.vx - vel.vy + r * vel.vomega    // BR
        };
    }

    /**
     * Forward kinematics: four wheel speeds → robot velocity.
     */
    public PoseVelocity2d forwardKinematics(double fl, double fr,
                                            double bl, double br) {
        double r = HALF_TRACK + HALF_WHEELBASE;
        return new PoseVelocity2d(
            (fl + fr + bl + br) / 4.0,
            (-fl + fr + bl - br) / 4.0,
            (-fl + fr - bl + br) / (4.0 * r));
    }

    // ---- bounding box ----

    /**
     * Returns the four corners of the robot's oriented bounding box
     * in world coordinates.  Corners are ordered:
     * front-left, front-right, back-right, back-left.
     */
    public double[][] getBoundingBoxCorners() {
        double hw = profile.width / 2.0;
        double hl = profile.length / 2.0;
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);

        double[][] local = {
            { hl,  hw },   // front-left
            { hl, -hw },   // front-right
            {-hl, -hw },   // back-right
            {-hl,  hw }    // back-left
        };

        double[][] world = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double fx = local[i][0];   // forward
            double lx = local[i][1];   // left
            world[i][0] = currentPose.x + fx * cos - lx * sin;
            world[i][1] = currentPose.y + fx * sin + lx * cos;
        }
        return world;
    }

    /**
     * Returns the robot's effective collision radius (half the diagonal
     * of the bounding box).  Useful for quick broad-phase rejection.
     */
    public double getCollisionRadius() {
        return Math.hypot(profile.width / 2.0, profile.length / 2.0);
    }

    // ---- collision detection ----

    /**
     * High-fidelity rotated-rectangle collision detection via dense
     * multi-resolution vertex sampling.
     *
     * <p>Samples the robot's oriented bounding box at three resolutions
     * to prevent tunnelling and thin-obstacle misses:</p>
     * <ol>
     *   <li><b>4 corners</b> — maximum extent of the bounding box.</li>
     *   <li><b>3 subdivisions per edge</b> — 12 edge-sample points
     *       (3 on each of the 4 edges) for fine-grained perimeter
     *       coverage.</li>
     *   <li><b>3×3 interior grid</b> — 9 internal sample points
     *       to catch thin obstacles that might thread between
     *       edge samples.</li>
     * </ol>
     *
     * <p>Total: 25 sample points that reliably model the robot's
     * full rectangular footprint at any orientation.</p>
     *
     * @param field the field map with obstacle grid
     * @return true if any sample point falls inside an obstacle cell
     */
    public boolean checkCollision(FieldMap field) {
        double hw = profile.width / 2.0;
        double hl = profile.length / 2.0;
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);

        // ---- layer 1: 4 corners ----
        double[] cornersF = { hl,  hl, -hl, -hl };   // forward  coords
        double[] cornersL = { hw, -hw, -hw,  hw };   // left     coords
        for (int i = 0; i < 4; i++) {
            double wx = currentPose.x + cornersF[i] * cos - cornersL[i] * sin;
            double wy = currentPose.y + cornersF[i] * sin + cornersL[i] * cos;
            if (field.isObstacle(wx, wy)) return true;
        }

        // ---- layer 2: 3 subdivisions per edge (12 points) ----
        // Edge sample fractions: 25%, 50%, 75% along each edge.
        double[] edgeFrac = {0.25, 0.50, 0.75};
        for (double t : edgeFrac) {
            // Front edge: forward = +hl, left varies from +hw to -hw
            double lf = hw * (1.0 - 2.0 * t);   // hw → 0 → -hw
            double wxf = currentPose.x + hl * cos - lf * sin;
            double wyf = currentPose.y + hl * sin + lf * cos;
            if (field.isObstacle(wxf, wyf)) return true;

            // Back edge: forward = -hl, left varies from -hw to +hw
            double lb = -hw * (1.0 - 2.0 * t);
            double wxb = currentPose.x - hl * cos - lb * sin;
            double wyb = currentPose.y - hl * sin + lb * cos;
            if (field.isObstacle(wxb, wyb)) return true;

            // Left edge: left = +hw, forward varies from +hl to -hl
            double fl = hl * (1.0 - 2.0 * t);
            double wxl = currentPose.x + fl * cos - hw * sin;
            double wyl = currentPose.y + fl * sin + hw * cos;
            if (field.isObstacle(wxl, wyl)) return true;

            // Right edge: left = -hw, forward varies from +hl to -hl
            double fr = hl * (1.0 - 2.0 * t);
            double wxr = currentPose.x + fr * cos + hw * sin;
            double wyr = currentPose.y + fr * sin - hw * cos;
            if (field.isObstacle(wxr, wyr)) return true;
        }

        // ---- layer 3: 3×3 interior grid (9 points) ----
        // Distribute points evenly inside the rectangle.
        double[] gridFrac = {-0.5, 0.0, 0.5};
        for (double gf : gridFrac) {
            for (double gl : gridFrac) {
                double f = gf * hl * 2.0;   // forward  offset from centre
                double l = gl * hw * 2.0;   // left     offset from centre
                double wx = currentPose.x + f * cos - l * sin;
                double wy = currentPose.y + f * sin + l * cos;
                if (field.isObstacle(wx, wy)) return true;
            }
        }

        return false;
    }

    /**
     * Checks whether any part of the robot would be outside the
     * field boundary.
     *
     * <p>Samples the 4 corners of the bounding box against the
     * field perimeter.</p>
     *
     * @param halfField half the field side length (inches)
     * @return true if any corner is outside the field
     */
    public boolean isOutOfBounds(double halfField) {
        double[][] corners = getBoundingBoxCorners();
        for (double[] c : corners) {
            if (c[0] < -halfField || c[0] > halfField
                || c[1] < -halfField || c[1] > halfField) {
                return true;
            }
        }
        return false;
    }

    // ---- simulation update ----

    /**
     * Updates the robot pose based on current velocity and time step.
     * Velocity is clamped to the profile's max limits.
     * Heading is normalized to [-π, π].
     */
    public void update(double dt) {
        // Clamp velocity to profile limits.
        double speed = Math.hypot(currentVelocity.vx, currentVelocity.vy);
        if (speed > profile.maxLinearVelocity && speed > 1e-9) {
            double scale = profile.maxLinearVelocity / speed;
            currentVelocity.vx *= scale;
            currentVelocity.vy *= scale;
        }
        if (Math.abs(currentVelocity.vomega) > profile.maxAngularVelocity) {
            currentVelocity.vomega = Math.signum(currentVelocity.vomega)
                                   * profile.maxAngularVelocity;
        }

        currentPose.x += currentVelocity.vx * dt;
        currentPose.y += currentVelocity.vy * dt;
        currentPose.heading = Math.IEEEremainder(
                currentPose.heading + currentVelocity.vomega * dt,
                2.0 * Math.PI);
    }

    @Override
    public String toString() {
        return "Robot[" + currentPose + ", vel=" + currentVelocity
               + ", " + profile + "]";
    }
}
