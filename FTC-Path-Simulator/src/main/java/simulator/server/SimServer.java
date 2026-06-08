package simulator.server;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

import simulator.algorithm.PathFinder;
import simulator.engine.AutoSimulationEngine;
import simulator.io.JavaCodeParser;
import simulator.model.*;
import simulator.model.FieldMap.FieldElement;
import simulator.physics.BallProjectileEngine;

import java.io.*;
import java.net.*;
import java.nio.file.Path;
import java.util.*;

/**
 * Headless TCP server that runs the FTC simulation and streams frame state
 * to a Metal renderer frontend via newline-delimited JSON.
 *
 * <p>Protocol (JSON lines, one message per '\\n'):</p>
 *
 * <b>Java → Renderer:</b>
 * <pre>
 * {"t":"ready"}
 * {"t":"status","txt":"..."}
 * {"t":"fld","els":[...]}       // sent once on connect
 * {"t":"frm","e":1.5,...}        // per-frame
 * {"t":"end","txt":"..."}
 * {"t":"err","txt":"..."}
 * </pre>
 *
 * <b>Renderer → Java:</b>
 * <pre>
 * {"cmd":"cfg","w":18,...}
 * {"cmd":"prog","txt":"MOVE_TO(24,24)\\n..."}
 * {"cmd":"start"}
 * {"cmd":"reset"}
 * {"cmd":"upload","path":"/path/to/OpMode.java"}
 * </pre>
 */
public class SimServer {

    private static final int DEFAULT_PORT = 9876;
    private static final double TICK_RATE = 1.0 / 60.0;   // 60 Hz

    private final int port;
    private final Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().create();

    // simulation
    private final FieldMap field;
    private RobotProfile profile;
    private Robot robot;
    private AutoSimulationEngine engine;
    private final FrameState frameState = new FrameState();
    private String savedProgram = "";   // persisted across resets

    private volatile boolean running;

    public SimServer() {
        this(DEFAULT_PORT);
    }

    public SimServer(int port) {
        this.port = port;
        this.field = new FieldMap();
        this.profile = new RobotProfile();
        this.robot = new Robot(profile, new Pose2d(-48, 48, 0));
        this.engine = new AutoSimulationEngine(robot, field);
    }

    public void start() throws IOException {
        try (ServerSocket serverSocket = new ServerSocket(port)) {
            System.out.println("[SimServer] Listening on port " + port);
            while (true) {
                Socket socket = serverSocket.accept();
                System.out.println("[SimServer] Renderer connected from "
                                   + socket.getInetAddress());
                handleClient(socket);
            }
        }
    }

    // ---- per-client session ----

    private void handleClient(Socket socket) {
        running = false;
        try (socket;
             BufferedReader in  = new BufferedReader(
                 new InputStreamReader(socket.getInputStream()));
             BufferedWriter out = new BufferedWriter(
                 new OutputStreamWriter(socket.getOutputStream()))) {

            send(out, msg("t", "ready"));
            sendFieldElements(out);
            sendStatus(out, "[READY] Awaiting configuration from renderer.");

            String line;
            while ((line = in.readLine()) != null) {
                processCommand(line.trim(), out);
            }
        } catch (IOException e) {
            System.err.println("[SimServer] Client disconnected: " + e.getMessage());
        }
        running = false;
    }

    // ---- command dispatch ----

    @SuppressWarnings("unchecked")
    private void processCommand(String line, BufferedWriter out) {
        if (line.isEmpty()) return;
        try {
            Map<String, Object> cmd = gson.fromJson(
                line, new TypeToken<Map<String, Object>>(){}.getType());
            String type = (String) cmd.get("cmd");
            if (type == null) return;

            switch (type) {
                case "cfg"    -> handleConfigure(cmd, out);
                case "prog"   -> handleLoadProgram(cmd, out);
                case "start"  -> handleStart(out);
                case "reset"  -> handleReset(out);
                case "upload" -> handleUpload(cmd, out);
                default -> sendStatus(out, "[WARN] Unknown command: " + type);
            }
        } catch (Exception e) {
            send(out, msg("err", "Parse error: " + e.getMessage()));
        }
    }

    // ---- command handlers ----

    private void handleConfigure(Map<String, Object> cmd, BufferedWriter out) {
        if (running) return;  // ignore during simulation
        try {
            double w  = getDouble(cmd, "w", 18.0);
            double l  = getDouble(cmd, "l", 18.0);
            double h  = getDouble(cmd, "h", 8.0);
            String cm = (String) cmd.getOrDefault("cm", "GOBILDA_5203_19_2");
            double wr = getDouble(cmd, "wr", 2.0);
            String lm = (String) cmd.getOrDefault("lm", "GOBILDA_5203_19_2");
            double la = getDouble(cmd, "la", 45.0);
            double lw = getDouble(cmd, "lw", 1.5);
            boolean vs = getBoolean(cmd, "vs", false);
            String it = (String) cmd.getOrDefault("it", "NONE");
            String sc = (String) cmd.getOrDefault("sc", "NONE");

            profile = new RobotProfile(w, l, h,
                MotorType.valueOf(cm), wr,
                MotorType.valueOf(lm), la, lw,
                vs,
                RobotProfile.IntakeType.valueOf(it),
                RobotProfile.ScoringCapability.valueOf(sc));
            robot = new Robot(profile, new Pose2d(-48, 48, 0));
            engine = new AutoSimulationEngine(robot, field);

            sendStatus(out, "[OK] Configured: " + profile);
        } catch (Exception e) {
            send(out, msg("err", "Config error: " + e.getMessage()));
        }
    }

    private void handleLoadProgram(Map<String, Object> cmd, BufferedWriter out) {
        String txt = (String) cmd.get("txt");
        if (txt == null || txt.isBlank()) {
            sendStatus(out, "[WARN] Empty program.");
            return;
        }
        savedProgram = txt;
        engine.loadProgram(txt);
        sendStatus(out, "[OK] Program loaded: " + engine.getLog().size() + " parse entries.");
    }

    private void handleStart(BufferedWriter out) {
        if (running) return;
        // Reset and reload saved program
        engine.reset();
        if (!savedProgram.isBlank()) {
            engine.loadProgram(savedProgram);
        }

        running = true;
        sendStatus(out, "[RUNNING] Simulation started.");

        // Run simulation on a background thread so the client handler
        // can still process reset commands.
        Thread simThread = new Thread(() -> {
            long lastNanos = System.nanoTime();
            while (running && !engine.isFinished()) {
                long now = System.nanoTime();
                double dt = (now - lastNanos) / 1_000_000_000.0;
                if (dt < TICK_RATE) {
                    try { Thread.sleep(1); } catch (InterruptedException e) { break; }
                    continue;
                }
                lastNanos = now;

                engine.tick(Math.min(dt, 0.05));
                frameState.fill(engine.getElapsedTime(),
                                engine.isFinished(),
                                engine.getStatusText(),
                                robot.currentPose,
                                engine.getTrail(),
                                engine.getCurrentPath(),
                                engine.getBallEngine());
                synchronized (out) {
                    send(out, gson.toJson(frameState));
                }

                if (engine.isFinished()) break;
            }
            running = false;
            synchronized (out) {
                send(out, msg("end", engine.getStatusText()));
            }
        }, "sim-loop");
        simThread.setDaemon(true);
        simThread.start();
    }

    private void handleReset(BufferedWriter out) {
        running = false;
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        this.robot = new Robot(profile, new Pose2d(-48, 48, 0));
        this.engine = new AutoSimulationEngine(robot, field);
        sendStatus(out, "[READY] Simulation reset.");
    }

    private void handleUpload(Map<String, Object> cmd, BufferedWriter out) {
        String pathStr = (String) cmd.get("path");
        if (pathStr == null) {
            sendStatus(out, "[ERROR] No path provided.");
            return;
        }
        try {
            List<String> cmds = new JavaCodeParser().parseFile(Path.of(pathStr));
            if (cmds.isEmpty()) {
                sendStatus(out, "[WARN] No FTC commands detected in file.");
                return;
            }
            StringBuilder sb = new StringBuilder();
            for (String c : cmds) sb.append(c).append('\n');
            engine.loadProgram(sb.toString().trim());
            sendStatus(out, "[OK] Loaded " + cmds.size() + " commands from " + pathStr);
        } catch (IOException e) {
            send(out, msg("err", "Failed to read file: " + e.getMessage()));
        }
    }

    // ---- helpers ----

    private double getDouble(Map<String, Object> m, String key, double def) {
        Object v = m.get(key);
        if (v instanceof Number n) return n.doubleValue();
        return def;
    }

    private boolean getBoolean(Map<String, Object> m, String key, boolean def) {
        Object v = m.get(key);
        if (v instanceof Boolean b) return b;
        return def;
    }

    // ---- wire output ----

    private void send(BufferedWriter out, String json) {
        try {
            out.write(json);
            out.newLine();
            out.flush();
        } catch (IOException e) {
            System.err.println("[SimServer] Write error: " + e.getMessage());
        }
    }

    private void sendStatus(BufferedWriter out, String text) {
        send(out, gson.toJson(Map.of("t", "status", "txt", text)));
    }

    private void sendFieldElements(BufferedWriter out) {
        List<Map<String, Object>> els = new ArrayList<>();
        for (FieldElement el : field.getElements()) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("tp", el.type.name());
            m.put("x",  (el.minX + el.maxX) / 2.0);
            m.put("y",  (el.minY + el.maxY) / 2.0);
            m.put("w",  el.maxX - el.minX);
            m.put("d",  el.maxY - el.minY);
            m.put("h",  el.height > 0.05 ? el.height : 0.15);
            m.put("c",  el.color.toRGBA());
            m.put("lb", el.label);
            els.add(m);
        }
        // manually build JSON to keep the "t" field
        Map<String, Object> wrapper = new LinkedHashMap<>();
        wrapper.put("t", "fld");
        wrapper.put("els", els);
        send(out, gson.toJson(wrapper));
    }

    private static String msg(String key, String value) {
        return String.format("{\"t\":\"%s\",\"txt\":%s}",
            key, escapeJson(value));
    }

    private static String escapeJson(String s) {
        StringBuilder sb = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            switch (c) {
                case '"'  -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default   -> sb.append(c);
            }
        }
        return sb.append('"').toString();
    }

    // ---- entry point ----

    public static void main(String[] args) {
        int port = DEFAULT_PORT;
        if (args.length > 0) {
            try { port = Integer.parseInt(args[0]); } catch (NumberFormatException ignored) {}
        }
        try {
            new SimServer(port).start();
        } catch (IOException e) {
            System.err.println("[SimServer] Failed to start: " + e.getMessage());
            System.exit(1);
        }
    }
}
