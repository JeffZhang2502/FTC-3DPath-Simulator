package simulator.io;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/**
 * Sends FTC Java source code to an AI API and extracts simulator commands.
 *
 * <p>Uses the OpenAI-compatible chat completions format.  Works with
 * OpenAI, Anthropic, Groq, Ollama, or any compatible endpoint.</p>
 *
 * <p>When no API key is configured the analysis is skipped.</p>
 */
public class AIAnalyzer {

    private String apiKey;
    private String endpoint;
    private String model;

    public AIAnalyzer() {
        this.endpoint = "https://api.openai.com/v1/chat/completions";
        this.model = "gpt-4o-mini";
    }

    public void setApiKey(String k)  { this.apiKey = k; }
    public void setEndpoint(String e) { this.endpoint = e; }
    public void setModel(String m)    { this.model = m; }
    public boolean isReady()          { return apiKey != null && !apiKey.isBlank(); }

    /**
     * Analyse FTC Java source and return simulator commands.
     * Returns an error message as the first entry on failure.
     */
    public List<String> analyze(String source) {
        if (!isReady()) return List.of();

        String prompt = """
You are an FTC robotics simulator. Convert the following FTC Java OpMode
into simulator commands.  Output ONLY the commands, one per line — no
explanations, no markdown, no code fences.

Available commands:
  MOVE_TO(x, y)   — absolute field position in inches (field is 144×144")
  WAIT(seconds)   — pause
  INTAKE          — activate intake / claw close
  SCORE           — score on obelisk / classifier
  LAUNCH          — shoot launcher
  DETECT          — vision sensor detection
  # comment       — any comment

Rules:
- If the code is @TeleOp (gamepad-driven), output ONLY:
  # TELEOP — requires gamepad input, cannot auto-simulate.
- For @Autonomous, extract the sequence from runOpMode().
- Convert mm to inches (÷25.4) when inferring distances.
- Convert milliseconds to seconds for WAIT.
- For time-based movement (move at power X for Y ms), estimate
  distance as: power × 48 in/s × seconds.  Emit MOVE_TO for
  forward/strafe and a comment for rotation.
- For odometry-based X(mm)/Y(mm) calls, convert to inches.
- Infer scoring actions from method names like fang(), score(), etc.

FTC Java source:
""" + source;

        try {
            String json = callApi(prompt);
            return extractCommands(json);
        } catch (Exception e) {
            return List.of("# AI error: " + e.getMessage());
        }
    }

    private String callApi(String prompt) throws Exception {
        String body = """
            {
              "model": "%s",
              "messages": [{"role": "user", "content": "%s"}],
              "temperature": 0.1,
              "max_tokens": 800
            }
            """.formatted(model, escape(prompt));

        HttpURLConnection c = (HttpURLConnection) URI.create(endpoint).toURL().openConnection();
        c.setRequestMethod("POST");
        c.setRequestProperty("Content-Type", "application/json");
        c.setRequestProperty("Authorization", "Bearer " + apiKey);
        c.setDoOutput(true);
        c.setConnectTimeout(20_000);
        c.setReadTimeout(30_000);

        try (OutputStream os = c.getOutputStream()) {
            os.write(body.getBytes(StandardCharsets.UTF_8));
        }

        int status = c.getResponseCode();
        try (Scanner s = new Scanner(
                status >= 200 && status < 300 ? c.getInputStream() : c.getErrorStream(),
                StandardCharsets.UTF_8).useDelimiter("\\A")) {
            String raw = s.hasNext() ? s.next() : "";
            if (status >= 300) throw new RuntimeException("HTTP " + status + ": " + raw);
            return raw;
        }
    }

    private List<String> extractCommands(String json) {
        List<String> out = new ArrayList<>();
        // Find "content" field in OpenAI chat completion response.
        int i = json.indexOf("\"content\"");
        if (i < 0) return out;
        i = json.indexOf('"', i + 10) + 1;

        StringBuilder content = new StringBuilder();
        for (; i < json.length(); i++) {
            char ch = json.charAt(i);
            if (ch == '\\' && i + 1 < json.length()) {
                char nxt = json.charAt(++i);
                content.append(switch (nxt) {
                    case 'n' -> '\n'; case 't' -> '\t';
                    case '"' -> '"'; case '\\' -> '\\';
                    default -> { content.append('\\'); yield nxt; }
                });
            } else if (ch == '"') break;
            else content.append(ch);
        }

        for (String line : content.toString().split("\\R")) {
            String t = line.trim();
            if (t.isEmpty() || t.equals("```") || t.startsWith("```")) continue;
            out.add(t);
        }
        return out;
    }

    private static String escape(String s) {
        StringBuilder sb = new StringBuilder();
        for (char c : s.toCharArray()) {
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default -> sb.append(c);
            }
        }
        return sb.toString();
    }
}
