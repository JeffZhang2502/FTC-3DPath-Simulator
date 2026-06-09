package simulator.io;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Scans a real FTC Java OpMode file and extracts autonomous actions
 * into human-readable auto-program lines consumable by
 * {@code AutoSimulationEngine}.
 *
 * <p>Supports a wide range of real-world FTC coding styles:</p>
 * <ul>
 *   <li>Road Runner trajectory API</li>
 *   <li>Standard FTC SDK (DcMotor, Servo, sleep)</li>
 *   <li>FTCLib command-based patterns</li>
 *   <li>Simple helper-method styles</li>
 * </ul>
 */
public class JavaCodeParser {

    // ================================================================
    //  ROAD RUNNER  —  trajectory builder
    // ================================================================

    /** .splineTo(new Vector2d(24, 0), ...)  or  .splineTo(24, 0, ...) */
    private static final Pattern SPLINE_TO = Pattern.compile(
        "\\.splineTo\\s*\\(\\s*(?:new\\s+Vector2d\\s*\\()?"
        + "\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)?",
        Pattern.CASE_INSENSITIVE);

    /** .lineTo(new Vector2d(24, 0))  or  .lineTo(24, 0) */
    private static final Pattern LINE_TO = Pattern.compile(
        "\\.lineTo\\s*\\(\\s*(?:new\\s+Vector2d\\s*\\()?"
        + "\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)?",
        Pattern.CASE_INSENSITIVE);

    /** .lineToX(48) / .lineToY(-12) */
    private static final Pattern LINE_TO_AXIS = Pattern.compile(
        "\\.lineTo([XY])\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** .setTangent(Math.toRadians(90)) */
    private static final Pattern SET_TANGENT = Pattern.compile(
        "\\.setTangent\\s*\\(\\s*(?:Math\\.toRadians\\s*\\()?"
        + "\\s*(-?[\\d.]+)\\s*\\)?",
        Pattern.CASE_INSENSITIVE);

    /** .forward(24) / .back(12) */
    private static final Pattern STRAIGHT = Pattern.compile(
        "\\.(forward|back)\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** .strafeLeft(12) / .strafeRight(12) */
    private static final Pattern STRAFE = Pattern.compile(
        "\\.(strafeLeft|strafeRight)\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** .turn(Math.toRadians(90)) / .turn(90) */
    private static final Pattern TURN = Pattern.compile(
        "\\.turn\\s*\\(\\s*(?:Math\\.toRadians\\s*\\()?"
        + "\\s*(-?[\\d.]+)\\s*\\)?",
        Pattern.CASE_INSENSITIVE);

    /** .waitSeconds(1.5) — Road Runner wait */
    private static final Pattern WAIT_SECONDS = Pattern.compile(
        "\\.waitSeconds\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  GENERIC MOVE-TO
    // ================================================================

    /** moveTo(x, y)  or  drive.moveTo(x, y)  or  robot.moveTo(x, y) */
    private static final Pattern MOVE_TO = Pattern.compile(
        "(?:\\w+\\.)?moveTo\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** driveTo(x, y)  — alternative naming */
    private static final Pattern DRIVE_TO = Pattern.compile(
        "(?:\\w+\\.)?driveTo\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** goto(x, y) */
    private static final Pattern GOTO = Pattern.compile(
        "(?:\\w+\\.)?goto\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  CUSTOM ODOMETRY MOVEMENT  —  X(int mm), Y(int mm), H(int deg)
    // ================================================================

    /** X(600) — custom odometry-based forward movement (mm).
     *  Converts mm → inches (÷25.4). */
    private static final Pattern CUSTOM_X = Pattern.compile(
        "\\bX\\s*\\(\\s*(-?[\\d.]+)\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    /** Y(40) — custom strafe movement (mm). */
    private static final Pattern CUSTOM_Y = Pattern.compile(
        "\\bY\\s*\\(\\s*(-?[\\d.]+)\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    /** H(-45) — custom heading rotation (degrees). */
    private static final Pattern CUSTOM_H = Pattern.compile(
        "\\bH\\s*\\(\\s*(-?[\\d.]+)\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    /** arc(angle, radius) — custom arc movement. */
    private static final Pattern CUSTOM_ARC = Pattern.compile(
        "\\barc\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    /** moveMecanum(drive, strafe, turn) — common timed movement primitive */
    private static final Pattern MOVE_MECANUM = Pattern.compile(
        "\\bmoveMecanum\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** Scoring sequence methods: fang(), jan(), jian(), gua(), qu() */
    private static final Pattern SCORING_SEQ = Pattern.compile(
        "\\b(?:fang|jan|jian|gua|qu)\\s*\\(\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    /** clawopen() / clawclose() — intake control */
    private static final Pattern CLAW_OPEN = Pattern.compile(
        "\\bclawopen\\s*\\(\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);
    private static final Pattern CLAW_CLOSE = Pattern.compile(
        "\\bclawclose\\s*\\(\\s*\\)\\s*;",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  MOTOR / DRIVE COMMANDS
    // ================================================================

    /** .setTargetPosition(1000)  — encoder drive */
    private static final Pattern SET_TARGET = Pattern.compile(
        "\\.setTargetPosition\\s*\\(\\s*(-?\\d+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** .setPower(0.8)  — any motor/component */
    private static final Pattern SET_POWER = Pattern.compile(
        "(\\w+)\\.setPower\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** runToPosition() */
    private static final Pattern RUN_TO_POS = Pattern.compile(
        "\\.setMode\\s*\\(\\s*\\w+\\.RUN_TO_POSITION\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  LAUNCH / SHOOT / FIRE
    // ================================================================

    /** shoot() / fire() / launch() / launcher.shoot() etc. */
    private static final Pattern SHOOT = Pattern.compile(
        "(?:\\w+\\.)?(?:shoot|fire|launch|score)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** launcher.setPower(0.9) — treat as launch if power ≥ 0.5 */
    private static final Pattern LAUNCHER_POWER = Pattern.compile(
        "launcher\\.setPower\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  INTAKE
    // ================================================================

    /** intake.on() / intake.activate() / intake.setPower() */
    private static final Pattern INTAKE_ON = Pattern.compile(
        "intake\\.(?:on|activate|setPower|start|collect|grab|close)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** intakeMotor.setPower(1) */
    private static final Pattern INTAKE_MOTOR = Pattern.compile(
        "intake\\w*\\.setPower\\s*\\(\\s*(1(?:\\.0)?|[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** claw.close() / claw.grab() */
    private static final Pattern CLAW = Pattern.compile(
        "(?:claw|gripper|grabber)\\.(?:close|grab|collect)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** intake.open() / intake.release() / claw.open() — treat as stop */
    private static final Pattern INTAKE_OFF = Pattern.compile(
        "(?:intake|claw|gripper)\\.(?:open|release|stop|off|retract)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  SLEEP / WAIT / DELAY
    // ================================================================

    /** sleep(1000) */
    private static final Pattern SLEEP = Pattern.compile(
        "sleep\\s*\\(\\s*(\\d+(?:\\.\\d+)?)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** wait(1000) */
    private static final Pattern WAIT = Pattern.compile(
        "\\bwait\\s*\\(\\s*(\\d+(?:\\.\\d+)?)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** delay(1000) */
    private static final Pattern DELAY = Pattern.compile(
        "delay\\s*\\(\\s*(\\d+(?:\\.\\d+)?)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ================================================================
    //  TRAJECTORY SEQUENCE NAME DETECTION
    // ================================================================

    /** trajectorySequenceBuilder / trajectoryActionBuilder */
    private static final Pattern TRAJ_BUILDER = Pattern.compile(
        "trajectory(?:Sequence|Action)Builder\\s*\\(\\s*(\\w+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** followTrajectory / followTrajectorySequence */
    private static final Pattern FOLLOW_TRAJ = Pattern.compile(
        "followTrajectory(?:Sequence)?\\s*\\(\\s*\"?([^\")]+)\"?\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    /** Pose / startPose: new Pose2d(x, y, heading) */
    private static final Pattern START_POSE = Pattern.compile(
        "(?:startPose|pose|Pose2d)\\s*=?\\s*new\\s+\\w+\\s*"
        + "\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*",
        Pattern.CASE_INSENSITIVE);

    // ---- moveMecanum state (instance fields shared with flushMecanumMove) ----
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private double lastDrive, lastStrafe, lastTurn;
    private boolean hasPendingMove;

    // ================================================================
    //  PUBLIC API
    // ================================================================

    /**
     * Reads a .java file and extracts FTC commands into simulator lines.
     *
     * @param filePath path to the FTC OpMode .java file
     * @return list of simulator command strings
     */
    public List<String> parseFile(Path filePath) throws IOException {
        String content = Files.readString(filePath);
        return parseContent(content);
    }

    /**
     * Parses FTC Java source text directly.
     */
    public List<String> parseContent(String source) {
        List<String> commands = new ArrayList<>();
        String[] lines = source.split("\\R");

        // State tracking — instance fields so helper methods can access them.
        this.lastX = Double.NaN;
        this.lastY = Double.NaN;
        this.lastDrive = 0;
        this.lastStrafe = 0;
        this.lastTurn = 0;
        this.hasPendingMove = false;
        double msVal, secVal;      // reused across sleep/wait/delay parsing
        boolean inTrajectory = false;

        for (String rawLine : lines) {
            String line = rawLine.trim();
            if (line.isEmpty()) continue;
            if (line.startsWith("//") || line.startsWith("/*") || line.startsWith("*")) continue;
            if (line.startsWith("@")) continue;   // annotations

            // ---- skip hardware init noise ----
            if (line.contains("hardwareMap.get(") || line.contains("hardwareMap.get(")) continue;
            if (line.contains(".setDirection(") || line.contains(".setMode(")) continue;
            if (line.contains("setZeroPowerBehavior(") || line.contains(".setZeroPowerBehavior")) continue;
            if (line.contains("telemetry.") || line.contains("telemetry.")) continue;
            if (line.matches(".*\\bodo\\.[a-z].*") || line.contains("GoBildaPinpoint")) continue;
            if (line.contains("imu.") || line.contains("IMU")) continue;
            if (line.contains(".resetYaw()") || line.contains("recalibrateIMU")) continue;
            if (line.contains("private ") || line.contains("public class ")) continue;
            if (line.equals("{") || line.equals("}")) continue;

            // ---- ROAD RUNNER splineTo ----
            Matcher msp = SPLINE_TO.matcher(line);
            if (msp.find()) {
                double x = Double.parseDouble(msp.group(1));
                double y = Double.parseDouble(msp.group(2));
                commands.add(String.format("MOVE_TO(%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            // ---- ROAD RUNNER lineTo ----
            Matcher ml = LINE_TO.matcher(line);
            if (ml.find()) {
                double x = Double.parseDouble(ml.group(1));
                double y = Double.parseDouble(ml.group(2));
                commands.add(String.format("MOVE_TO(%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            // ---- ROAD RUNNER lineToX / lineToY ----
            Matcher mlAxis = LINE_TO_AXIS.matcher(line);
            if (mlAxis.find()) {
                String axis = mlAxis.group(1);
                double val = Double.parseDouble(mlAxis.group(2));
                if ("X".equalsIgnoreCase(axis)) {
                    commands.add(String.format("MOVE_TO(%.1f, %.1f)",
                        val, Double.isNaN(lastY) ? 0 : lastY));
                    lastX = val;
                } else {
                    commands.add(String.format("MOVE_TO(%.1f, %.1f)",
                        Double.isNaN(lastX) ? 0 : lastX, val));
                    lastY = val;
                }
                continue;
            }

            // ---- .forward() / .back() — relative movement ----
            Matcher mf = STRAIGHT.matcher(line);
            if (mf.find()) {
                double dist = Double.parseDouble(mf.group(2));
                String dir = mf.group(1).toLowerCase();
                String note = dir.equals("back") ? "backward" : "forward";
                commands.add(String.format("# %s %.1f inches (relative)", note, dist));
                continue;
            }

            // ---- .strafeLeft() / .strafeRight() ----
            Matcher mst = STRAFE.matcher(line);
            if (mst.find()) {
                double dist = Double.parseDouble(mst.group(2));
                String dir = mst.group(1).toLowerCase();
                commands.add(String.format("# strafe %s %.1f inches (relative)", dir, dist));
                continue;
            }

            // ---- .turn() ----
            Matcher mt = TURN.matcher(line);
            if (mt.find()) {
                double deg = Double.parseDouble(mt.group(1));
                commands.add(String.format("# turn %.0f degrees", deg));
                continue;
            }

            // ---- Custom odometry X(mm) → MOVE_TO (inches) ----
            Matcher mx = CUSTOM_X.matcher(line);
            if (mx.find()) {
                double mm = Double.parseDouble(mx.group(1));
                double inches = mm / 25.4;
                commands.add(String.format("MOVE_TO(%.1f, %.1f)",
                    inches, Double.isNaN(lastY) ? 0 : lastY));
                lastX = inches;
                continue;
            }

            // ---- Custom odometry Y(mm) → MOVE_TO (inches) ----
            Matcher my = CUSTOM_Y.matcher(line);
            if (my.find()) {
                double mm = Double.parseDouble(my.group(1));
                double inches = mm / 25.4;
                commands.add(String.format("MOVE_TO(%.1f, %.1f)",
                    Double.isNaN(lastX) ? 0 : lastX, inches));
                lastY = inches;
                continue;
            }

            // ---- Custom heading H(deg) ----
            Matcher mh = CUSTOM_H.matcher(line);
            if (mh.find()) {
                double deg = Double.parseDouble(mh.group(1));
                commands.add(String.format("# rotate to %d°", (int) deg));
                continue;
            }

            // ---- Custom arc(angle, radius) ----
            Matcher ma = CUSTOM_ARC.matcher(line);
            if (ma.find()) {
                double angle = Double.parseDouble(ma.group(1));
                double radius = Double.parseDouble(ma.group(2));
                commands.add(String.format("# arc %.0f° radius %.0f\"", angle, radius));
                continue;
            }

            // ---- moveMecanum(drive, strafe, turn) — store for sleep pairing ----
            Matcher mmec = MOVE_MECANUM.matcher(line);
            if (mmec.find()) {
                // Flush any previous pending move before storing new one.
                if (hasPendingMove) {
                    flushMecanumMove(commands, 0);
                }
                lastDrive  = Double.parseDouble(mmec.group(1));
                lastStrafe = Double.parseDouble(mmec.group(2));
                lastTurn   = Double.parseDouble(mmec.group(3));
                hasPendingMove = true;
                // If all zero → brake; flush a brake comment.
                if (lastDrive == 0 && lastStrafe == 0 && lastTurn == 0) {
                    hasPendingMove = false;
                }
                continue;
            }

            // ---- Custom scoring sequences → SCORE ----
            if (SCORING_SEQ.matcher(line).find()) {
                commands.add("SCORE");
                continue;
            }

            // ---- Claw open/close ----
            if (CLAW_CLOSE.matcher(line).find()) {
                commands.add("INTAKE");
                continue;
            }
            if (CLAW_OPEN.matcher(line).find()) {
                commands.add("# claw released");
                continue;
            }

            // ---- .waitSeconds() ----
            Matcher mws = WAIT_SECONDS.matcher(line);
            if (mws.find()) {
                double sec = Double.parseDouble(mws.group(1));
                commands.add(String.format("WAIT(%.2f)", sec));
                continue;
            }

            // ---- .setTangent() ----
            if (SET_TANGENT.matcher(line).find()) {
                continue;   // tangent is metadata, skip
            }

            // ---- trajectoryBuilder ----
            if (TRAJ_BUILDER.matcher(line).find()) {
                inTrajectory = true;
                commands.add("# --- trajectory building ---");
                continue;
            }

            // ---- .build() — end of trajectory ----
            if (line.matches(".*\\.build\\s*\\(\\s*\\).*")) {
                inTrajectory = false;
                continue;
            }

            // ---- followTrajectory ----
            Matcher mft = FOLLOW_TRAJ.matcher(line);
            if (mft.find()) {
                commands.add("# Trajectory: " + mft.group(1));
                continue;
            }

            // ---- MOVE_TO / driveTo / goto ----
            Matcher mm = MOVE_TO.matcher(line);
            if (mm.find()) {
                double x = Double.parseDouble(mm.group(1));
                double y = Double.parseDouble(mm.group(2));
                commands.add(String.format("MOVE_TO(%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            Matcher md = DRIVE_TO.matcher(line);
            if (md.find()) {
                double x = Double.parseDouble(md.group(1));
                double y = Double.parseDouble(md.group(2));
                commands.add(String.format("MOVE_TO(%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            Matcher mg = GOTO.matcher(line);
            if (mg.find()) {
                double x = Double.parseDouble(mg.group(1));
                double y = Double.parseDouble(mg.group(2));
                commands.add(String.format("MOVE_TO(%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            // ---- START_POSE ----
            Matcher mstart = START_POSE.matcher(line);
            if (mstart.find()) {
                double x = Double.parseDouble(mstart.group(1));
                double y = Double.parseDouble(mstart.group(2));
                commands.add(String.format("# start pose: (%.1f, %.1f)", x, y));
                lastX = x; lastY = y;
                continue;
            }

            // ---- .setTargetPosition / encoder commands ----
            Matcher mstp = SET_TARGET.matcher(line);
            if (mstp.find()) {
                int ticks = Integer.parseInt(mstp.group(1));
                commands.add(String.format("# encoder target: %d ticks", ticks));
                continue;
            }

            if (RUN_TO_POS.matcher(line).find()) {
                continue;   // metadata
            }

            // ---- SET_POWER — classify by variable name ----
            Matcher mspw = SET_POWER.matcher(line);
            if (mspw.find()) {
                String name = mspw.group(1).toLowerCase();
                double power = Double.parseDouble(mspw.group(2));
                if (name.contains("intake")) {
                    commands.add("INTAKE");
                } else if (name.contains("launch") || name.contains("shoot")) {
                    if (power > 0.5) commands.add("LAUNCH");
                } else if (name.contains("arm") || name.contains("lift")) {
                    commands.add(String.format("# %s set to %.1f%%", name, power * 100));
                } else if (name.contains("drive") || name.contains("motor")) {
                    // ignore drive motor commands — navigation handles movement
                }
                continue;
            }

            // ---- LAUNCH / SHOOT / FIRE ----
            if (SHOOT.matcher(line).find()) {
                commands.add("LAUNCH");
                continue;
            }

            // ---- LAUNCHER power ----
            Matcher mlp = LAUNCHER_POWER.matcher(line);
            if (mlp.find()) {
                double power = Double.parseDouble(mlp.group(1));
                if (power > 0.5) {
                    commands.add("LAUNCH");
                }
                continue;
            }

            // ---- INTAKE commands ----
            if (INTAKE_ON.matcher(line).find()) {
                commands.add("INTAKE");
                continue;
            }
            if (INTAKE_MOTOR.matcher(line).find()) {
                commands.add("INTAKE");
                continue;
            }
            if (CLAW.matcher(line).find()) {
                commands.add("INTAKE");
                continue;
            }
            if (INTAKE_OFF.matcher(line).find()) {
                commands.add("# intake released");
                continue;
            }

            // ---- SLEEP / WAIT / DELAY (with moveMecanum pairing) ----
            Matcher msl = SLEEP.matcher(line);
            if (msl.find()) {
                msVal = Double.parseDouble(msl.group(1));
                secVal = msVal > 100 ? msVal / 1000.0 : msVal;
                flushMecanumMove(commands, secVal);
                continue;
            }

            Matcher mw = WAIT.matcher(line);
            if (mw.find()) {
                msVal = Double.parseDouble(mw.group(1));
                secVal = msVal > 100 ? msVal / 1000.0 : msVal;
                flushMecanumMove(commands, secVal);
                continue;
            }

            Matcher mdy = DELAY.matcher(line);
            if (mdy.find()) {
                msVal = Double.parseDouble(mdy.group(1));
                secVal = msVal > 100 ? msVal / 1000.0 : msVal;
                flushMecanumMove(commands, secVal);
                continue;
            }

            // ---- SCORE ----
            if (line.matches(".*\\bscore\\s*\\(\\s*\\).*")) {
                commands.add("SCORE");
                continue;
            }

            // ---- DETECT ----
            if (line.matches(".*\\bdetect\\s*\\(\\s*\\).*")
                || line.contains("vision.") || line.contains("AprilTag")) {
                commands.add("DETECT");
                continue;
            }
        }

        // Flush any pending moveMecanum before returning.
        if (hasPendingMove) {
            flushMecanumMove(commands, 0);
        }
        System.out.println("[PARSER] Extracted " + commands.size() + " commands:");
        for (String c : commands) System.out.println("  " + c);
        return commands;
    }

    // ---- moveMecanum → MOVE_TO estimation ----

    /** Estimated max linear speed at power=1.0 (inches/sec).  Tune this. */
    private static final double MAX_SPEED_IPS = 48.0;

    /**
     * Converts a pending {@code moveMecanum(drive, strafe, turn)} +
     * {@code sleep(ms)} pair into a relative MOVE_TO command.
     *
     * @param commands  output list to append to
     * @param duration  sleep duration in seconds (0 = flush without wait)
     */
    private void flushMecanumMove(List<String> commands, double duration) {
        if (!hasPendingMove) {
            if (duration > 0) {
                commands.add(String.format("WAIT(%.2f)", duration));
            }
            return;
        }

        double dx = lastDrive  * MAX_SPEED_IPS * duration;
        double dy = lastStrafe * MAX_SPEED_IPS * duration;
        // Turn is rotational — emit as a comment for now.
        if (Math.abs(lastTurn) > 0.001) {
            double deg = lastTurn * 180.0 * duration;   // rough: 0.5 turn × 1s ≈ 90°
            commands.add(String.format("# rotate %.0f° over %.2fs (turn=%.2f)",
                deg, duration, lastTurn));
        }

        double nx = (Double.isNaN(lastX) ? 0 : lastX) + dx;
        double ny = (Double.isNaN(lastY) ? 0 : lastY) + dy;

        if (Math.abs(dx) > 0.5 || Math.abs(dy) > 0.5) {
            commands.add(String.format("MOVE_TO(%.1f, %.1f)", nx, ny));
            lastX = nx;
            lastY = ny;
        }
        if (duration > 0 && Math.abs(lastTurn) < 0.001) {
            commands.add(String.format("WAIT(%.2f)", duration));
        }

        hasPendingMove = false;
        lastDrive = lastStrafe = lastTurn = 0;
    }
}
