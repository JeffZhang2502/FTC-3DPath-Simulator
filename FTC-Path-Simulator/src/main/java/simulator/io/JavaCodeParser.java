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
 */
public class JavaCodeParser {

    // ---- regex patterns ----

    private static final Pattern MOVE_TO = Pattern.compile(
        "moveTo\\s*\\(\\s*(-?[\\d.]+)\\s*,\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    private static final Pattern FOLLOW_TRAJECTORY = Pattern.compile(
        "followTrajectory(?:Sequence)?\\s*\\(\\s*\"?([^\")]+)\"?\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    private static final Pattern SHOOT = Pattern.compile(
        "(?:shoot|fire|launch)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    private static final Pattern LAUNCHER_POWER = Pattern.compile(
        "launcher\\.setPower\\s*\\(\\s*(-?[\\d.]+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    private static final Pattern SLEEP = Pattern.compile(
        "sleep\\s*\\(\\s*(\\d+)\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    private static final Pattern INTAKE_ON = Pattern.compile(
        "intake\\.(?:on|setPower|activate)\\s*\\(\\s*\\)",
        Pattern.CASE_INSENSITIVE);

    // ---- public API ----

    /**
     * Reads a .java file and extracts FTC commands into simulator lines.
     *
     * @param filePath path to the FTC OpMode .java file
     * @return list of simulator command strings, e.g. {@code ["MOVE_TO(24, -12)", "INTAKE", ...]}
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

        for (String line : lines) {
            String trimmed = line.trim();
            if (trimmed.isEmpty() || trimmed.startsWith("//") || trimmed.startsWith("/*"))
                continue;

            // MOVE_TO / drive.followTrajectory
            Matcher mm = MOVE_TO.matcher(trimmed);
            if (mm.find()) {
                commands.add(String.format("MOVE_TO(%s, %s)", mm.group(1), mm.group(2)));
                continue;
            }

            Matcher mt = FOLLOW_TRAJECTORY.matcher(trimmed);
            if (mt.find()) {
                // Heuristic: trajectory name may contain waypoint coords.
                // Fall back to a placeholder; the user can edit manually.
                commands.add("# Trajectory: " + mt.group(1));
                continue;
            }

            // Launcher / shoot
            Matcher ml = LAUNCHER_POWER.matcher(trimmed);
            if (ml.find()) {
                double power = Double.parseDouble(ml.group(1));
                if (power > 0.5) {
                    commands.add("LAUNCH");
                }
                continue;
            }

            if (SHOOT.matcher(trimmed).find()) {
                commands.add("LAUNCH");
                continue;
            }

            // Sleep → WAIT
            Matcher ms = SLEEP.matcher(trimmed);
            if (ms.find()) {
                double sec = Integer.parseInt(ms.group(1)) / 1000.0;
                commands.add(String.format("WAIT(%.2f)", sec));
                continue;
            }

            // Intake
            if (INTAKE_ON.matcher(trimmed).find()) {
                commands.add("INTAKE");
            }
        }
        return commands;
    }
}
