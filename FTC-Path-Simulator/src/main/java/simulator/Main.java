package simulator;

import simulator.server.SimServer;

/**
 * FTC Auto Simulator — headless entry point.
 *
 * <p>Starts a TCP server on port 9876 (or the port given as argv[0])
 * that accepts connections from a Metal/Swift frontend renderer.
 * The simulation engine runs headless and streams frame-state JSON
 * over the socket in real time.</p>
 *
 * <p>Usage:</p>
 * <pre>
 *   ./gradlew run              # default port 9876
 *   java ... simulator.Main 8080   # custom port
 * </pre>
 */
public class Main {

    public static void main(String[] args) {
        System.out.println("=== FTC Auto Simulator — Headless Server ===");
        System.out.println("Waiting for Metal renderer connection...");
        SimServer.main(args);
    }
}
