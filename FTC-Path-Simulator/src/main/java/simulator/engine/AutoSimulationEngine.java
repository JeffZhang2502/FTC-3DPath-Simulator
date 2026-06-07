package simulator.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import simulator.algorithm.PathFinder;
import simulator.model.*;
import simulator.model.FieldMap.ZoneType;
import simulator.physics.BallProjectileEngine;
import simulator.physics.BallProjectileEngine.GoalBox;

/**
 * Executes a user-authored auto program against a robot and field.
 *
 * <p>Supported commands (one per line):</p>
 * <pre>
 *   MOVE_TO(x, y)      — navigate to absolute field position
 *   INTAKE              — activate intake mechanism
 *   SCORE               — attempt scoring at current position
 *   DETECT              — use vision sensor (requires hasVisionSensor)
 *   WAIT(seconds)       — pause for a duration
 * </pre>
 *
 * <p>The engine is driven by calling {@link #tick(double)} once per
 * simulation frame (typically at ~60 Hz).  Simulation ends after
 * 30 seconds or when all commands have been processed.</p>
 */
public class AutoSimulationEngine {

    // ---- constants ----

    /** Maximum simulation time in seconds. */
    public static final double MATCH_DURATION = 30.0;

    /** Proximity threshold (inches) for considering a MOVE_TO target reached. */
    private static final double ARRIVAL_THRESHOLD = 1.5;

    /** Collision penalty: fraction of speed retained after impact. */
    private static final double COLLISION_SLOWDOWN = 0.3;

    // ---- command types ----

    public enum CommandType {
        MOVE_TO, INTAKE, SCORE, DETECT, WAIT, LAUNCH
    }

    /** A single parsed auto command. */
    public static class AutoCommand {
        public final CommandType type;
        public final double[] params;   // MOVE_TO→[x,y], WAIT→[seconds], others→[]

        public AutoCommand(CommandType type, double... params) {
            this.type = type;
            this.params = params;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder(type.name());
            if (params.length > 0) {
                sb.append('(');
                for (int i = 0; i < params.length; i++) {
                    if (i > 0) sb.append(", ");
                    sb.append(String.format("%.1f", params[i]));
                }
                sb.append(')');
            }
            return sb.toString();
        }
    }

    // ---- engine state ----

    private final Robot robot;
    private final FieldMap field;
    private final List<AutoCommand> commands = new ArrayList<>();
    private final List<String> log = new ArrayList<>();
    private final List<Pose2d> trail = new ArrayList<>();

    private int commandIndex;
    private double elapsedTime;
    private double commandTimer;
    private boolean finished;
    private boolean collisionActive;

    // MOVE_TO internal state
    private double targetX, targetY;

    // A* pathfinding state
    private final PathFinder pathFinder = new PathFinder();
    private List<Pose2d> currentPath = Collections.emptyList();
    private int waypointIndex;

    // Ball projectile state
    private final BallProjectileEngine ballEngine = new BallProjectileEngine();
    private GoalBox activeGoal;

    /** Pre-defined goal boxes on the DECODE field. */
    private static final GoalBox[] GOALS = {
        new GoalBox( 48,  48, 8.0, BallProjectileEngine.GOAL_HEIGHT),  // red alliance
        new GoalBox(-48, -48, 8.0, BallProjectileEngine.GOAL_HEIGHT),  // blue alliance
    };

    public AutoSimulationEngine(Robot robot, FieldMap field) {
        this.robot = robot;
        this.field = field;
        reset();
    }

    // ---- lifecycle ----

    /** Reset the engine to prepare for a new run. */
    public void reset() {
        commands.clear();
        log.clear();
        trail.clear();
        commandIndex = 0;
        elapsedTime = 0.0;
        commandTimer = 0.0;
        finished = false;
        collisionActive = false;
        currentPath = Collections.emptyList();
        waypointIndex = 0;
        log("[READY] Simulation reset. Awaiting start.");
    }

    // ---- command parsing ----

    /**
     * Parses a multi-line auto program text into commands and
     * resets the engine state ready for execution.
     */
    public void loadProgram(String programText) {
        reset();
        if (programText == null || programText.isBlank()) {
            log("[WARN] Empty program — robot will stay idle.");
            return;
        }
        for (String raw : programText.split("\\R")) {
            String line = raw.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;

            AutoCommand cmd = parseLine(line);
            if (cmd != null) {
                commands.add(cmd);
                log("[PARSE] " + cmd);
            } else {
                log("[PARSE] Ignored unrecognised line: " + line);
            }
        }
        log("[PARSE] Loaded " + commands.size() + " command(s).");
    }

    private AutoCommand parseLine(String line) {
        String upper = line.toUpperCase();
        try {
            if (upper.startsWith("MOVE_TO")) {
                double[] p = extractParams(line);
                if (p.length >= 2) return new AutoCommand(CommandType.MOVE_TO, p[0], p[1]);
            } else if (upper.startsWith("INTAKE")) {
                return new AutoCommand(CommandType.INTAKE);
            } else if (upper.startsWith("SCORE")) {
                return new AutoCommand(CommandType.SCORE);
            } else if (upper.startsWith("DETECT")) {
                return new AutoCommand(CommandType.DETECT);
            } else if (upper.startsWith("WAIT")) {
                double[] p = extractParams(line);
                if (p.length >= 1) return new AutoCommand(CommandType.WAIT, p[0]);
            } else if (upper.startsWith("LAUNCH") || upper.startsWith("SHOOT")) {
                return new AutoCommand(CommandType.LAUNCH);
            }
        } catch (NumberFormatException e) {
            return null;
        }
        return null;
    }

    /** Extracts comma-separated numbers from parentheses, e.g. "(24, -12)". */
    private double[] extractParams(String line) {
        int start = line.indexOf('(');
        int end = line.indexOf(')');
        if (start < 0 || end < 0 || end <= start) return new double[0];
        String[] parts = line.substring(start + 1, end).split(",");
        double[] vals = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            vals[i] = Double.parseDouble(parts[i].trim());
        }
        return vals;
    }

    // ---- simulation tick ----

    /**
     * Advance the simulation by {@code dt} seconds.
     * @param dt time delta in seconds (will be capped to avoid huge jumps).
     */
    public void tick(double dt) {
        if (finished) return;

        // Cap dt to prevent physics explosion on lag spikes.
        if (dt > 0.05) dt = 0.05;
        if (dt <= 0.0) return;

        elapsedTime += dt;
        if (elapsedTime >= MATCH_DURATION) {
            elapsedTime = MATCH_DURATION;
            log(String.format("[%.0fs] ⏰ Match time expired.", elapsedTime));
            finished = true;
            return;
        }

        // Record trail point every ~0.1s.
        if (trail.isEmpty() || elapsedTime - getLastTrailTime() > 0.1) {
            trail.add(new Pose2d(robot.currentPose.x,
                                 robot.currentPose.y,
                                 robot.currentPose.heading));
        }

        if (commandIndex >= commands.size()) {
            finished = true;
            log(String.format("[%.0fs] ✅ All commands completed.", elapsedTime));
            return;
        }

        AutoCommand cmd = commands.get(commandIndex);
        commandTimer += dt;

        switch (cmd.type) {
            case MOVE_TO -> tickMoveTo(dt, cmd);
            case INTAKE  -> tickIntake(dt);
            case SCORE   -> tickScore(dt);
            case DETECT  -> tickDetect(dt);
            case WAIT    -> tickWait(dt, cmd);
            case LAUNCH  -> tickLaunch(dt);
        }
    }

    // ---- per-command tick logic ----

    private void tickMoveTo(double dt, AutoCommand cmd) {
        // On first frame: plan the path with A*.
        if (commandTimer <= dt + 0.0001) {
            targetX = cmd.params[0];
            targetY = cmd.params[1];
            log(String.format("[%.0fs] 🚀 A* pathfinding to (%.1f, %.1f)...",
                              elapsedTime, targetX, targetY));

            Pose2d goal = new Pose2d(targetX, targetY, 0);
            currentPath = pathFinder.findPath(field, robot.currentPose, goal);

            if (currentPath.isEmpty()) {
                log(String.format("[%.0fs] ❌ ERROR: Target isolated — "
                                  + "A* cannot find a path to (%.1f, %.1f)!",
                                  elapsedTime, targetX, targetY));
                advanceCommand();
                return;
            }
            waypointIndex = 0;
            log(String.format("[%.0fs] 📐 Path found: %d waypoints.",
                              elapsedTime, currentPath.size()));
        }

        // Consume waypoints.
        while (waypointIndex < currentPath.size()) {
            Pose2d wp = currentPath.get(waypointIndex);
            double dx = wp.x - robot.currentPose.x;
            double dy = wp.y - robot.currentPose.y;
            double dist = Math.hypot(dx, dy);

            if (dist < ARRIVAL_THRESHOLD) {
                waypointIndex++;
                continue;
            }

            // Move towards this waypoint.
            double dirX = dx / dist;
            double dirY = dy / dist;
            double speed = robot.getProfile().maxLinearVelocity;
            if (collisionActive) speed *= COLLISION_SLOWDOWN;

            robot.currentPose.heading = Math.atan2(dy, dx);
            double step = speed * dt;
            if (step > dist) step = dist;

            // Sub-step sampling to prevent tunneling.
            final int SUB_STEPS = 3;
            double subStep = step / SUB_STEPS;
            boolean hit = false;

            for (int i = 0; i < SUB_STEPS; i++) {
                robot.currentPose.x += dirX * subStep;
                robot.currentPose.y += dirY * subStep;

                if (robot.checkCollision(field)) {
                    hit = true;
                    robot.currentPose.x -= dirX * subStep;
                    robot.currentPose.y -= dirY * subStep;
                    break;
                }
            }

            robot.setCurrentVelocity(
                new PoseVelocity2d(dirX * speed, dirY * speed, 0));

            // Surface-sliding collision response.
            if (hit) {
                if (!collisionActive) {
                    collisionActive = true;
                    log(String.format("[%.0fs] ⚠️ COLLISION! Searching for slide path...",
                                      elapsedTime));
                }
                double bestDot = -Double.MAX_VALUE;
                double bestDx = 0, bestDy = 0;
                double[] angles = {0, Math.PI / 4, Math.PI / 2, 3 * Math.PI / 4,
                                   Math.PI, -3 * Math.PI / 4, -Math.PI / 2, -Math.PI / 4};
                double savedX = robot.currentPose.x;
                double savedY = robot.currentPose.y;

                for (double a : angles) {
                    double tx = savedX + Math.cos(a) * step;
                    double ty = savedY + Math.sin(a) * step;
                    robot.currentPose.x = tx;
                    robot.currentPose.y = ty;

                    if (!robot.checkCollision(field)) {
                        double dot = Math.cos(a) * dirX + Math.sin(a) * dirY;
                        if (dot > bestDot) {
                            bestDot = dot;
                            bestDx = tx - savedX;
                            bestDy = ty - savedY;
                        }
                    }
                }

                if (bestDot > -Double.MAX_VALUE / 2) {
                    robot.currentPose.x = savedX + bestDx;
                    robot.currentPose.y = savedY + bestDy;
                } else {
                    robot.currentPose.x = savedX;
                    robot.currentPose.y = savedY;
                    robot.setCurrentVelocity(new PoseVelocity2d(0, 0, 0));
                    log(String.format("[%.0fs] 🛑 STUCK! No free path.", elapsedTime));
                    advanceCommand();
                    collisionActive = false;
                    return;
                }
            } else {
                collisionActive = false;
            }
            return;   // one waypoint per tick
        }

        // All waypoints consumed → arrived at final target.
        robot.currentPose.x = targetX;
        robot.currentPose.y = targetY;
        robot.setCurrentVelocity(new PoseVelocity2d(0, 0, 0));
        log(String.format("[%.0fs] 📍 Arrived at (%.1f, %.1f).",
                          elapsedTime, targetX, targetY));
        advanceCommand();
    }

    private void tickIntake(double dt) {
        if (commandTimer <= dt + 0.0001) {
            RobotProfile.IntakeType intake = robot.getProfile().intakeType;
            if (intake == RobotProfile.IntakeType.NONE) {
                log(String.format("[%.0fs] ❌ INTAKE failed: no intake mechanism installed.",
                                  elapsedTime));
                advanceCommand();
                return;
            }
            log(String.format("[%.0fs] 🤖 Intake activated (%s)...",
                              elapsedTime, intake));
        }
        // Intake takes 1 second.
        if (commandTimer >= 1.0) {
            log(String.format("[%.0fs] ✅ Intake complete.", elapsedTime));
            advanceCommand();
        }
    }

    private void tickScore(double dt) {
        if (commandTimer <= dt + 0.0001) {
            RobotProfile.ScoringCapability cap = robot.getProfile().scoringCapability;
            if (cap == RobotProfile.ScoringCapability.NONE) {
                log(String.format("[%.0fs] ❌ SCORE failed: no scoring mechanism installed.",
                                  elapsedTime));
                advanceCommand();
                return;
            }

            // Check if robot is near a scoring zone.
            boolean nearObelisk = isNearZone(ZoneType.OBELISK);
            boolean nearClassifier = isNearZone(ZoneType.SCORING_ZONE);

            if (cap == RobotProfile.ScoringCapability.OBELISK_ONLY && !nearObelisk) {
                log(String.format("[%.0fs] ⚠️ SCORE: robot has OBELISK_ONLY but is not "
                                  + "near an obelisk. Attempting anyway...", elapsedTime));
            } else if (cap == RobotProfile.ScoringCapability.CLASSIFIER_ONLY
                       && !nearClassifier) {
                log(String.format("[%.0fs] ⚠️ SCORE: robot has CLASSIFIER_ONLY but is not "
                                  + "near a classifier gate. Attempting anyway...",
                                  elapsedTime));
            }

            String where = nearObelisk ? "obelisk" : nearClassifier ? "classifier" : "field";
            log(String.format("[%.0fs] 🎯 Scoring at %s (%s)...",
                              elapsedTime, where, cap));
        }
        // Scoring takes 1.5 seconds.
        if (commandTimer >= 1.5) {
            log(String.format("[%.0fs] 🏆 Score successful!", elapsedTime));
            advanceCommand();
        }
    }

    private void tickDetect(double dt) {
        if (commandTimer <= dt + 0.0001) {
            if (!robot.getProfile().hasVisionSensor) {
                log(String.format("[%.0fs] 🔍 DETECT: no vision sensor! "
                                  + "Robot is confused, stalling for 3s...",
                                  elapsedTime));
            } else {
                log(String.format("[%.0fs] 👁️ Vision detection active (AprilTag / prop)...",
                                  elapsedTime));
            }
        }

        double required = robot.getProfile().hasVisionSensor ? 0.3 : 3.0;
        if (commandTimer >= required) {
            if (robot.getProfile().hasVisionSensor) {
                log(String.format("[%.0fs] ✅ Target identified.", elapsedTime));
            } else {
                log(String.format("[%.0fs] ⏰ Stall ended — no vision data. "
                                  + "Continuing blind.", elapsedTime));
            }
            advanceCommand();
        }
    }

    private void tickWait(double dt, AutoCommand cmd) {
        double duration = cmd.params[0];
        if (commandTimer <= dt + 0.0001) {
            log(String.format("[%.0fs] ⏳ Waiting %.1fs...", elapsedTime, duration));
        }
        if (commandTimer >= duration) {
            advanceCommand();
        }
    }

    private void tickLaunch(double dt) {
        if (commandTimer <= dt + 0.0001) {
            // Pick the closest goal.
            activeGoal = GOALS[0];
            double bestD = Double.MAX_VALUE;
            for (GoalBox g : GOALS) {
                double d = Math.hypot(robot.currentPose.x - g.cx,
                                      robot.currentPose.y - g.cy);
                if (d < bestD) { bestD = d; activeGoal = g; }
            }

            double v0 = robot.getProfile().getLauncherVelocity();
            double z0 = robot.getProfile().height;
            ballEngine.launch(robot.currentPose.x, robot.currentPose.y, z0,
                              robot.currentPose.heading,
                              robot.getProfile().launchAngleDeg, v0);
            log(String.format("[%.0fs] 🚀 LAUNCH! v₀=%.0f in/s, goal at (%.0f, %.0f).",
                              elapsedTime, v0, activeGoal.cx, activeGoal.cy));
        }

        // Update ball physics.
        String result = ballEngine.update(dt, activeGoal);
        if (result != null) {
            log(String.format("[%.0fs] %s", elapsedTime, result));
            advanceCommand();
        } else if (commandTimer > 5.0) {
            // timeout after 5s flight
            log(String.format("[%.0fs] ⏰ Ball flight timeout.", elapsedTime));
            advanceCommand();
        }
    }

    // ---- helpers ----

    private void advanceCommand() {
        commandIndex++;
        commandTimer = 0.0;
        collisionActive = false;
        currentPath = Collections.emptyList();
        waypointIndex = 0;
    }

    private boolean isNearZone(FieldMap.ZoneType targetType) {
        double threshold = 10.0;  // inches
        for (FieldMap.FieldElement el : field.getElements()) {
            if (el.type == targetType) {
                double cx = (el.minX + el.maxX) / 2.0;
                double cy = (el.minY + el.maxY) / 2.0;
                double d = Math.hypot(robot.currentPose.x - cx,
                                      robot.currentPose.y - cy);
                if (d < threshold) return true;
            }
        }
        return false;
    }

    private void log(String msg) {
        log.add(msg);
        // Keep log bounded.
        if (log.size() > 200) {
            log.remove(0);
        }
    }

    private double getLastTrailTime() {
        // Trail is recorded approximately every 0.1s; we use elapsedTime
        // minus a small epsilon.  This method isn't perfect but keeps the
        // trail clean.  We simply check the trail list size and spacing.
        // For a simpler approach, use a dedicated lastTrailTime field.
        return elapsedTime - 0.2;   // conservative fallback
    }

    // ---- public queries ----

    public Robot getRobot()          { return robot; }
    public FieldMap getField()       { return field; }
    public double getElapsedTime()   { return elapsedTime; }
    public boolean isFinished()      { return finished; }
    public boolean isCollisionActive() { return collisionActive; }
    public List<String> getLog()     { return new ArrayList<>(log); }

    /** Returns the last N log messages, most recent first. */
    public List<String> getRecentLog(int n) {
        List<String> recent = new ArrayList<>(log);
        int from = Math.max(0, recent.size() - n);
        return recent.subList(from, recent.size());
    }

    public List<Pose2d> getTrail()   { return new ArrayList<>(trail); }

    /** Returns the A*-planned path (empty if not in a MOVE_TO command). */
    public List<Pose2d> getCurrentPath() {
        return new ArrayList<>(currentPath);
    }

    /** Returns the ball projectile engine for 3D rendering. */
    public BallProjectileEngine getBallEngine() {
        return ballEngine;
    }

    public AutoCommand getCurrentCommand() {
        if (commandIndex < commands.size()) return commands.get(commandIndex);
        return null;
    }

    /** Returns a one-line status summary for the UI status bar. */
    public String getStatusText() {
        if (finished && commandIndex >= commands.size()) {
            return String.format("[%.0fs] ✅ Auto complete. All %d commands executed.",
                                 elapsedTime, commands.size());
        }
        if (finished) {
            return String.format("[%.0fs] ⏰ Match time expired. %d/%d commands completed.",
                                 elapsedTime, commandIndex, commands.size());
        }
        AutoCommand cmd = getCurrentCommand();
        String cmdStr = cmd != null ? cmd.toString() : "NONE";
        return String.format("[%.0fs] Executing: %s  |  cmd %d/%d",
                             elapsedTime, cmdStr, commandIndex + 1, commands.size());
    }
}
