package simulator.model;

/**
 * Real FTC motor hardware specifications.
 * RPM and torque values drive the simulation's velocity,
 * acceleration, and launcher physics limits.
 */
public enum MotorType {

    /** goBILDA 5203 series, 19.2:1 ratio — high speed. */
    GOBILDA_5203_19_2(312, 1.5, "goBILDA 19.2:1"),

    /** goBILDA 5203 series, 30:1 ratio — high torque. */
    GOBILDA_5203_30_1(196, 2.4, "goBILDA 30:1"),

    /** REV HD Hex 20:1. */
    REV_HD_HEX_20_1(300, 1.2, "REV HD Hex 20:1");

    /** No-load RPM at nominal voltage. */
    public final double rpm;

    /** Rated torque in N·m. */
    public final double torqueNm;

    /** Human-readable label. */
    public final String label;

    MotorType(double rpm, double torqueNm, String label) {
        this.rpm = rpm;
        this.torqueNm = torqueNm;
        this.label = label;
    }

    @Override
    public String toString() {
        return String.format("%s (%.0f RPM, %.1f N·m)", label, rpm, torqueNm);
    }
}
