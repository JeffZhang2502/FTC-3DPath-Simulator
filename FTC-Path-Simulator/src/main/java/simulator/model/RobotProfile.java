package simulator.model;

/**
 * User-configurable robot parameters including dimensions,
 * motor hardware, launcher specs, and functional capabilities.
 *
 * <p>All length units are in inches (standard FTC field convention).
 * Speed limits are derived from motor RPM and wheel radius.</p>
 */
public class RobotProfile {

    /** Intake / collector mechanism type. */
    public enum IntakeType {
        NONE, SERVO_CLAW, INTAKE_ROLLER
    }

    /** Scoring capability — which field elements the robot can score on. */
    public enum ScoringCapability {
        NONE, OBELISK_ONLY, CLASSIFIER_ONLY
    }

    // ---- dimensions (inches) ----

    public double width;
    public double length;
    public double height;         // 3D body height, default 8"

    // ---- chassis hardware ----

    public MotorType chassisMotor;
    public double wheelRadius;    // drive wheel radius, default 2"

    // ---- launcher hardware ----

    public MotorType launcherMotor;
    public double launchAngleDeg; // default 45°
    public double launcherWheelRadius; // flywheel radius, default 1.5"

    // ---- derived speed limits (computed from motor + wheel) ----

    public double maxLinearVelocity;
    public double maxAngularVelocity;

    // ---- functional features ----

    public boolean hasVisionSensor;
    public IntakeType intakeType;
    public ScoringCapability scoringCapability;

    // ---- constructors ----

    public RobotProfile() {
        this.width = 18.0;
        this.length = 18.0;
        this.height = 8.0;
        this.chassisMotor = MotorType.GOBILDA_5203_19_2;
        this.wheelRadius = 2.0;
        this.launcherMotor = MotorType.GOBILDA_5203_19_2;
        this.launchAngleDeg = 45.0;
        this.launcherWheelRadius = 1.5;
        this.hasVisionSensor = false;
        this.intakeType = IntakeType.NONE;
        this.scoringCapability = ScoringCapability.NONE;
        recomputeSpeedLimits();
    }

    public RobotProfile(double width, double length, double height,
                        MotorType chassisMotor, double wheelRadius,
                        MotorType launcherMotor, double launchAngleDeg,
                        double launcherWheelRadius,
                        boolean hasVisionSensor,
                        IntakeType intakeType,
                        ScoringCapability scoringCapability) {
        this.width = width;
        this.length = length;
        this.height = height;
        this.chassisMotor = chassisMotor;
        this.wheelRadius = wheelRadius;
        this.launcherMotor = launcherMotor;
        this.launchAngleDeg = launchAngleDeg;
        this.launcherWheelRadius = launcherWheelRadius;
        this.hasVisionSensor = hasVisionSensor;
        this.intakeType = intakeType;
        this.scoringCapability = scoringCapability;
        recomputeSpeedLimits();
    }

    // ---- physics helpers ----

    /** Recompute linear / angular speed caps from motor RPM + wheel radius. */
    public void recomputeSpeedLimits() {
        // v = ω * r  →  inches/sec = (rpm / 60) * 2π * radius
        double wheelRps = chassisMotor.rpm / 60.0;
        this.maxLinearVelocity = wheelRps * 2.0 * Math.PI * wheelRadius;
        // Angular: roughly half-track rate limited by differential wheel speed.
        this.maxAngularVelocity = (maxLinearVelocity / (width * 0.5)) * 0.7;
    }

    /**
     * Computes the ball launch velocity v₀ (inches/sec) from the launcher
     * motor RPM and flywheel radius.
     */
    public double getLauncherVelocity() {
        double rps = launcherMotor.rpm / 60.0;
        return rps * 2.0 * Math.PI * launcherWheelRadius;
    }

    @Override
    public String toString() {
        return String.format(
            "RobotProfile[%dx%dx%d in, chassis=%s, launcher=%s, "
            + "vMax=%.0f in/s, launch=%.0f°, vision=%s, intake=%s, score=%s]",
            width, length, height,
            chassisMotor.label, launcherMotor.label,
            maxLinearVelocity, launchAngleDeg,
            hasVisionSensor, intakeType, scoringCapability);
    }
}
