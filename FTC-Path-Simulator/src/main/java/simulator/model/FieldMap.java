package simulator.model;

import java.util.ArrayList;
import java.util.List;
import javafx.scene.paint.Color;

/**
 * Digital-twin representation of the 144×144 inch FTC DECODE field
 * built from the official Section 9 ARENA specifications.
 *
 * <p>Sources:</p>
 * <ul>
 *   <li>FTC DECODE Game Manual §9 — ARENA dimensions &amp; zones</li>
 *   <li>AndyMark FTC Perimeter Field Kit (am-2600b) — wall height 12.125"</li>
 *   <li>Onshape CAD assembly (c7b090d255194e764d0c133c)</li>
 * </ul>
 *
 * <p>Road Runner coordinate convention: field center = (0, 0),
 * X ∈ [-72, 72], Y ∈ [-72, 72] inches.  +Y = audience side,
 * −Y = goal side.</p>
 */
public class FieldMap {

    // ==================== constants from official manual ====================

    /** Field side length — §9.1 */
    public static final double FIELD_SIZE = 144.0;
    public static final double HALF_FIELD = 72.0;

    /** Tile size — §9.1: 36 interlocking 24"×24"×0.59" tiles */
    public static final double TILE_SIZE = 24.0;
    public static final int    TILES_PER_SIDE = 6;

    /** Perimeter wall height — AndyMark FTC field kit am-2600b */
    public static final double WALL_HEIGHT = 12.125;

    /** Obelisk — §9.6: equilateral triangular prism, 23" tall, 11" face */
    public static final double OBELISK_HEIGHT     = 23.0;
    public static final double OBELISK_FACE_WIDTH = 11.0;
    /** Distance from triangular face to prism centre: s·√3/6 */
    public static final double OBELISK_INRADIUS =
        OBELISK_FACE_WIDTH * Math.sqrt(3.0) / 6.0;   // ≈ 3.175"
    /** Circumradius: s·√3/3 ≈ 6.35" (for collision cylinder) */
    public static final double OBELISK_CIRCRADIUS =
        OBELISK_FACE_WIDTH * Math.sqrt(3.0) / 3.0;

    /** Obelisk centre X (centred on goal side) — §9.6 */
    public static final double OBELISK_CX = 0.0;
    /** Obelisk centre Y: one face contacts south wall at Y=−72,
     *  centre is −72 − inradius ≈ −75.175 */
    public static final double OBELISK_CY = -HALF_FIELD - OBELISK_INRADIUS;

    /** Goal rim height (standard FTC basketball-style goal). */
    public static final double GOAL_RIM_HEIGHT = 38.75;
    /** Goal inner radius — approximate basket opening. */
    public static final double GOAL_INNER_RADIUS = 8.0;

    /** Alliance area — §9.2: 96" wide × 54" deep */
    public static final double ALLIANCE_WIDTH = 96.0;
    public static final double ALLIANCE_DEPTH  = 54.0;

    /** Base zone — §9.3: 18"×18" */
    public static final double BASE_ZONE_SIZE = 18.0;

    // Goal centre positions (estimated from field symmetry — verify vs Onshape).
    public static final double GOAL_RED_CX  = -28.0;
    public static final double GOAL_RED_CY  = -60.0;
    public static final double GOAL_BLUE_CX =  28.0;
    public static final double GOAL_BLUE_CY = -60.0;

    // ==================== data structures ====================

    /** Grid resolution = 1 cell per inch. */
    public static final int GRID_SIZE = 144;

    private final boolean[][] obstacles;
    private final List<FieldElement> elements;

    // ==================== enums ====================

    public enum ZoneType {
        /** Solid obstacle (wall, divider). */
        OBSTACLE,
        RED_START,
        BLUE_START,
        /** Triangular prism — §9.6 */
        OBELISK,
        CLASSIFIER_BOX,
        CLASSIFIER_RAMP,
        SCORING_ZONE,
        /** Perimeter wall panels. */
        PERIMETER_WALL,
        /** Goal / basket assembly. */
        GOAL_ASSEMBLY
    }

    /** A labelled, coloured region or 3D element on the field. */
    public static class FieldElement {
        public final ZoneType type;
        public final double minX, minY, maxX, maxY;   // world inches (XY footprint)
        public final double height;                     // Z height in inches
        public final String label;
        public final Color color;

        public FieldElement(ZoneType type, double minX, double minY,
                            double maxX, double maxY, double height,
                            String label, Color color) {
            this.type = type; this.minX = minX; this.minY = minY;
            this.maxX = maxX; this.maxY = maxY; this.height = height;
            this.label = label; this.color = color;
        }

        /** Convenience: flat element (height = 0). */
        public FieldElement(ZoneType type, double minX, double minY,
                            double maxX, double maxY, String label, Color color) {
            this(type, minX, minY, maxX, maxY, 0.0, label, color);
        }
    }

    // ==================== constructor ====================

    public FieldMap() {
        this.obstacles = new boolean[GRID_SIZE][GRID_SIZE];
        this.elements = new ArrayList<>();
        placeDecodeField();
    }

    // ==================== coordinate helpers ====================

    public static int worldToGridX(double wx) {
        return clamp((int) Math.floor(wx + HALF_FIELD), 0, GRID_SIZE - 1);
    }
    public static int worldToGridY(double wy) {
        return clamp((int) Math.floor(wy + HALF_FIELD), 0, GRID_SIZE - 1);
    }
    private static int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }

    // ==================== obstacle queries ====================

    public boolean isObstacle(double wx, double wy) {
        int gx = worldToGridX(wx), gy = worldToGridY(wy);
        return obstacles[gx][gy];
    }
    public boolean isObstacleCell(int gx, int gy) {
        if (gx < 0 || gx >= GRID_SIZE || gy < 0 || gy >= GRID_SIZE) return false;
        return obstacles[gx][gy];
    }

    // ==================== ring / cylinder collision ====================

    /**
     * Checks whether a world point falls inside the obelisk's
     * triangular-prism collision volume (modelled as its circumcircle).
     */
    public boolean isInsideObelisk(double wx, double wy) {
        double dx = wx - OBELISK_CX;
        double dy = wy - OBELISK_CY;
        // Use circumradius for a conservative collision cylinder.
        return Math.hypot(dx, dy) <= OBELISK_CIRCRADIUS + 0.5;
    }

    /** Checks if a point is within the perimeter wall footprint. */
    public boolean isInsidePerimeterWall(double wx, double wy) {
        double wallHalfThick = 0.625;   // ~⅝" wall panel thickness
        // North wall
        if (wy >= HALF_FIELD - wallHalfThick && wy <= HALF_FIELD + wallHalfThick) return true;
        // South wall
        if (wy <= -HALF_FIELD + wallHalfThick && wy >= -HALF_FIELD - wallHalfThick) return true;
        // West wall
        if (wx <= -HALF_FIELD + wallHalfThick && wx >= -HALF_FIELD - wallHalfThick) return true;
        // East wall
        if (wx >= HALF_FIELD - wallHalfThick && wx <= HALF_FIELD + wallHalfThick) return true;
        return false;
    }

    // ==================== element access ====================

    public List<FieldElement> getElements() { return elements; }

    // ==================== fill helpers ====================

    private void fillRect(double minX, double minY, double maxX, double maxY,
                          boolean isObstacle, ZoneType type, String label,
                          Color color) {
        elements.add(new FieldElement(type, minX, minY, maxX, maxY,
                                      isObstacle ? WALL_HEIGHT : 0.0,
                                      label, color));
        if (!isObstacle) return;
        int gx0 = worldToGridX(minX), gy0 = worldToGridY(minY);
        int gx1 = worldToGridX(maxX), gy1 = worldToGridY(maxY);
        for (int gx = gx0; gx <= gx1; gx++)
            for (int gy = gy0; gy <= gy1; gy++)
                obstacles[gx][gy] = true;
    }

    /** Fill an element with explicit 3D height. */
    private void fillElement(double minX, double minY, double maxX, double maxY,
                             double height, boolean isObstacle,
                             ZoneType type, String label, Color color) {
        elements.add(new FieldElement(type, minX, minY, maxX, maxY,
                                      height, label, color));
        if (!isObstacle) return;
        int gx0 = worldToGridX(minX), gy0 = worldToGridY(minY);
        int gx1 = worldToGridX(maxX), gy1 = worldToGridY(maxY);
        for (int gx = gx0; gx <= gx1; gx++)
            for (int gy = gy0; gy <= gy1; gy++)
                obstacles[gx][gy] = true;
    }

    // ==================== DECODE field layout (§9 + CAD BOM) ====================
    //
    //  Verified against Parasolid am-5700 assembly BOM (94 instances, 51 parts):
    //    11 × am-2160b wall panels (48"×1.25"×12.125")  → 3N + 3W + 3E + 2S
    //     4 × am-2600b corner brackets  (3"×3"×12.125")
    //     1 × am-5715 + am-5716 obelisk  (11" triangular prism, 23" total)
    //     2 × am-5735 goal assemblies    (20"×20"×38.75")
    //     2 × am-5718 classifier boxes   (18"×12"×12")
    //     2 × am-5707 ramps              (24"×12"×6")
    //     3 × am-5704 gate assemblies
    //     2 × am-5731 alliance station bases

    private static final double WALL_PANEL_W = 48.0;
    private static final double WALL_THICK   = 1.25;    // am-2160b panel thickness
    private static final double CORNER_SIZE  = 3.0;     // am-2600b bracket
    private static final double GOAL_W       = 20.0;    // am-5735 width/depth
    private static final double CLASS_W      = 18.0;    // am-5718 classifier box
    private static final double CLASS_D      = 12.0;
    private static final double CLASS_H      = 12.0;
    private static final double RAMP_W       = 24.0;    // am-5707
    private static final double RAMP_D       = 12.0;
    private static final double RAMP_H       = 6.0;

    private void placeDecodeField() {

        // ===== Perimeter walls (11 panels × 48", 4 corners) =====
        // North wall: 3 panels → 0–144"  (full north side)
        placeWallPanel(-HALF_FIELD,           HALF_FIELD,            0);  // NW
        placeWallPanel(-HALF_FIELD + 48,      HALF_FIELD,            0);  // N-mid
        placeWallPanel(-HALF_FIELD + 96,      HALF_FIELD,            0);  // NE

        // West wall: 3 panels → full west side
        placeWallPanel(-HALF_FIELD - WALL_THICK, HALF_FIELD - 48,    90);  // W-top
        placeWallPanel(-HALF_FIELD - WALL_THICK, HALF_FIELD - 96,    90);  // W-mid
        placeWallPanel(-HALF_FIELD - WALL_THICK, -HALF_FIELD + 48,   90);  // W-bot

        // East wall: 3 panels → full east side
        placeWallPanel(HALF_FIELD,               HALF_FIELD - 48,    90);  // E-top
        placeWallPanel(HALF_FIELD,               HALF_FIELD - 96,    90);  // E-mid
        placeWallPanel(HALF_FIELD,               -HALF_FIELD + 48,   90);  // E-bot

        // South wall: 2 panels → 96" total, leaves 48" goal opening centered
        placeWallPanel(-HALF_FIELD,              -HALF_FIELD - WALL_THICK, 0); // SW
        placeWallPanel(-HALF_FIELD + 96,         -HALF_FIELD - WALL_THICK, 0); // SE
        // → gap from X=−24 to X=+24 is the 48" goal-side opening.

        // Corner brackets (am-2600b, 3"×3").
        double cs = CORNER_SIZE;
        fillRect(-HALF_FIELD - cs,  HALF_FIELD, -HALF_FIELD,  HALF_FIELD + cs, true,
                 ZoneType.OBSTACLE, "NW Corner", Color.rgb(140, 140, 145, 0.8));
        fillRect(-HALF_FIELD - cs, -HALF_FIELD - cs, -HALF_FIELD, -HALF_FIELD, true,
                 ZoneType.OBSTACLE, "SW Corner", Color.rgb(140, 140, 145, 0.8));
        fillRect(HALF_FIELD,  HALF_FIELD, HALF_FIELD + cs,  HALF_FIELD + cs, true,
                 ZoneType.OBSTACLE, "NE Corner", Color.rgb(140, 140, 145, 0.8));
        fillRect(HALF_FIELD, -HALF_FIELD - cs, HALF_FIELD + cs, -HALF_FIELD, true,
                 ZoneType.OBSTACLE, "SE Corner", Color.rgb(140, 140, 145, 0.8));

        // ===== Alliance Starting Zones (§9.2) =====
        fillRect(-72, 24, -24, 72, false, ZoneType.RED_START,
                 "Red Alliance Start", Color.rgb(255, 80, 80, 0.25));
        fillRect(24, 24, 72, 72, false, ZoneType.BLUE_START,
                 "Blue Alliance Start", Color.rgb(80, 80, 255, 0.25));

        // ===== Obelisk (§9.6 + am-5715/am-5716) =====
        //    Base 11"×11"×4" + Top 11" triangular prism × 19" = total 23"
        double or = OBELISK_CIRCRADIUS;
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            for (int gy = 0; gy < GRID_SIZE; gy++) {
                double wx = gx - HALF_FIELD + 0.5;
                double wy = gy - HALF_FIELD + 0.5;
                if (isInsideObelisk(wx, wy)) obstacles[gx][gy] = true;
            }
        }
        elements.add(new FieldElement(ZoneType.OBELISK,
                OBELISK_CX - or, OBELISK_CY - or,
                OBELISK_CX + or, OBELISK_CY + or,
                OBELISK_HEIGHT,
                "Obelisk (am-5715/5716, 23\" △ prism 11\" face)",
                Color.rgb(50, 50, 55, 0.9)));

        // ===== Goal assemblies (am-5735, 20"×20"×38.75") =====
        // Red goal: west side of the south opening
        addGoalAssembly(GOAL_RED_CX,  GOAL_RED_CY,  "Red Goal (am-5735)");
        // Blue goal: east side of the south opening
        addGoalAssembly(GOAL_BLUE_CX, GOAL_BLUE_CY, "Blue Goal (am-5735)");

        // ===== Classifier structures (am-5718 boxes + am-5707 ramps) =====
        // Red classifier: west side, near Red goal
        fillElement(-48, -72, -48 + CLASS_W, -72 + CLASS_D,
                     CLASS_H, true, ZoneType.CLASSIFIER_BOX,
             "Classifier Red (am-5718)", Color.rgb(200, 140, 60, 0.7));
        fillElement(-48 - RAMP_D, -72 + CLASS_D, -48, -72 + CLASS_D + RAMP_W,
                     RAMP_H, true, ZoneType.CLASSIFIER_RAMP,
             "Ramp Red (am-5707)", Color.rgb(180, 120, 40, 0.6));

        // Blue classifier: east side, near Blue goal
        fillElement(48 - CLASS_W, -72, 48, -72 + CLASS_D,
                     CLASS_H, true, ZoneType.CLASSIFIER_BOX,
             "Classifier Blue (am-5718)", Color.rgb(200, 140, 60, 0.7));
        fillElement(48, -72 + CLASS_D, 48 + RAMP_D, -72 + CLASS_D + RAMP_W,
                     RAMP_H, true, ZoneType.CLASSIFIER_RAMP,
             "Ramp Blue (am-5707)", Color.rgb(180, 120, 40, 0.6));

        // ===== Scoring zones (tape markers) =====
        fillRect(-48, -76, -20, -60, false, ZoneType.SCORING_ZONE,
                 "Scoring Zone (Red)",  Color.rgb(255, 215, 0, 0.3));
        fillRect(20, -76,  48, -60, false, ZoneType.SCORING_ZONE,
                 "Scoring Zone (Blue)", Color.rgb(255, 215, 0, 0.3));
    }

    /** Place a single 48" wall panel at (worldX, worldY) with the given
     *  orientation (0 = horizontal along X, 90 = vertical along Y). */
    private void placeWallPanel(double wx, double wy, int orientDeg) {
        double pw, pd;
        if (orientDeg == 0) { pw = WALL_PANEL_W; pd = WALL_THICK; }
        else                { pw = WALL_THICK;   pd = WALL_PANEL_W; }
        fillElement(wx, wy, wx + pw, wy + pd,
                     WALL_HEIGHT, true, ZoneType.PERIMETER_WALL,
                     "Wall Panel (am-2160b, 48\"×12.125\")",
                     Color.rgb(180, 180, 180, 0.7));
    }

    private void addGoalAssembly(double cx, double cy, String label) {
        double half = GOAL_W / 2.0;   // 10"
        fillElement(cx - half, cy - half, cx + half, cy + half,
                     GOAL_RIM_HEIGHT, true, ZoneType.GOAL_ASSEMBLY,
                     label + " (am-5735, " + GOAL_W + "\"×" + GOAL_W
                     + "\"×" + GOAL_RIM_HEIGHT + "\")",
                     Color.rgb(40, 40, 50, 0.8));
    }
}
