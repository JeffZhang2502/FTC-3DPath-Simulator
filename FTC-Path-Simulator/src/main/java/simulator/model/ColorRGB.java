package simulator.model;

/**
 * Simple RGBA colour holder, replacing {@code javafx.scene.paint.Color}.
 * Values: r/g/b in [0, 255], a in [0.0, 1.0].
 */
public class ColorRGB {

    public final double r, g, b, a;

    public ColorRGB(double r, double g, double b) {
        this(r, g, b, 1.0);
    }

    public ColorRGB(double r, double g, double b, double a) {
        this.r = clamp(r, 0, 255);
        this.g = clamp(g, 0, 255);
        this.b = clamp(b, 0, 255);
        this.a = clamp(a, 0, 1);
    }

    /** Pack to RGBA int for IPC wire format. */
    public int toRGBA() {
        int ri = (int) r;
        int gi = (int) g;
        int bi = (int) b;
        int ai = (int) (a * 255.0 + 0.5);
        return (ai << 24) | (ri << 16) | (gi << 8) | bi;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public String toString() {
        return String.format("rgba(%.0f,%.0f,%.0f,%.2f)", r, g, b, a);
    }
}
