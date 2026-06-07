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

        // Local coords: (forward, left) = (hl, hw) etc.
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

    // ---- collision detection ----

    /**
     * Four-corner vertex sampling with edge midpoints for robust
     * rotated collision detection.
     *
     * <p>Computes 8 sample points on the robot's oriented bounding box
     * (4 corners + 4 edge midpoints), rotates each into world coordinates
     * via the heading rotation matrix, and checks whether any point
     * falls inside an obstacle cell on the field map.</p>
     */
    public boolean checkCollision(FieldMap field) {
        double hw = profile.width / 2.0;
        double hl = profile.length / 2.0;
        double cos = Math.cos(currentPose.heading);
        double sin = Math.sin(currentPose.heading);

        // 8 sample points in local frame (forward, left).
        double[][] samples = {
            { hl,  hw },   // front-left  corner
            { hl, -hw },   // front-right corner
            {-hl, -hw },   // back-right  corner
            {-hl,  hw },   // back-left   corner
            { hl, 0.0 },   // front edge  midpoint
            {-hl, 0.0 },   // back  edge  midpoint
            {0.0,  hw },   // left  edge  midpoint
            {0.0, -hw }    // right edge  midpoint
        };

        for (double[] s : samples) {
            // Rotate local (forward, left) by heading.
            double wx = currentPose.x + s[0] * cos - s[1] * sin;
            double wy = currentPose.y + s[0] * sin + s[1] * cos;
            if (field.isObstacle(wx, wy)) {
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
