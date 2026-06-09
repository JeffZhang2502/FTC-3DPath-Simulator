package simulator.algorithm;

import java.util.*;
import simulator.model.FieldMap;
import simulator.model.Pose2d;

/**
 * A* pathfinding engine on the 144×144 inch field grid.
 *
 * <p>Supports 8-directional movement with straight-cost 1.0 and
 * diagonal-cost √2.  The heuristic is diagonal distance — admissible
 * and optimal for 8-connected grids.  Found paths are post-smoothed
 * via line-of-sight collapsing.</p>
 */
public class PathFinder {

    private static final double SQRT2 = 1.4142135623730951;
    private static final double SQRT2_MINUS_1 = SQRT2 - 1.0;

    /** 8-directional neighbour offsets: {dRow, dCol, cost}. */
    private static final int[][] DIRS = {
        {-1,  0,  1},          // N
        { 1,  0,  1},          // S
        { 0, -1,  1},          // W
        { 0,  1,  1},          // E
        {-1, -1,  1414},       // NW  (cost × 1000 to keep int)
        {-1,  1,  1414},       // NE
        { 1, -1,  1414},       // SW
        { 1,  1,  1414}        // SE
    };

    // ---- public API ----

    /**
     * Finds an optimal path from start to goal, avoiding obstacles.
     *
     * @param field the field map with obstacle data
     * @param start starting pose in world inches
     * @param goal  target pose in world inches
     * @return a smoothed list of waypoints (world inches), or an empty
     *         list if no path exists
     */
    public List<Pose2d> findPath(FieldMap field, Pose2d start, Pose2d goal) {
        int startRow = FieldMap.worldToGridY(start.y);
        int startCol = FieldMap.worldToGridX(start.x);
        int goalRow  = FieldMap.worldToGridY(goal.y);
        int goalCol  = FieldMap.worldToGridX(goal.x);

        // If start or goal is inside an obstacle, bail.
        if (field.isObstacleCell(startCol, startRow)
            || field.isObstacleCell(goalCol, goalRow)) {
            return Collections.emptyList();
        }

        // Already there?
        if (startRow == goalRow && startCol == goalCol) {
            return Collections.singletonList(new Pose2d(goal.x, goal.y, 0));
        }

        // ---- A* search ----
        PriorityQueue<Node> open = new PriorityQueue<>();
        boolean[][] closed = new boolean[FieldMap.GRID_SIZE][FieldMap.GRID_SIZE];

        Node startNode = new Node(startRow, startCol, 0,
                                   heuristic(startRow, startCol, goalRow, goalCol),
                                   null);
        open.add(startNode);

        Node goalNode = null;

        while (!open.isEmpty()) {
            Node cur = open.poll();
            int r = cur.row, c = cur.col;

            if (closed[r][c]) continue;
            closed[r][c] = true;

            if (r == goalRow && c == goalCol) {
                goalNode = cur;
                break;
            }

            for (int[] d : DIRS) {
                int nr = r + d[0];
                int nc = c + d[1];
                if (nr < 0 || nr >= FieldMap.GRID_SIZE
                    || nc < 0 || nc >= FieldMap.GRID_SIZE) continue;
                if (closed[nr][nc]) continue;
                if (field.isObstacleCell(nc, nr)) continue;

                // For diagonal moves, prevent corner-cutting through walls.
                if (d[2] == 1414) {
                    if (field.isObstacleCell(c, nr)     // horizontal neighbour
                        || field.isObstacleCell(nc, r)) // vertical neighbour
                        continue;
                }

                double stepCost = d[2] >= 1000 ? SQRT2 : 1.0;
                double ng = cur.g + stepCost;
                double nh = heuristic(nr, nc, goalRow, goalCol);

                open.add(new Node(nr, nc, ng, nh, cur));
            }
        }

        if (goalNode == null) {
            return Collections.emptyList();   // no path
        }

        // ---- reconstruct raw grid path ----
        List<Pose2d> raw = new ArrayList<>();
        Node n = goalNode;
        while (n != null) {
            raw.add(gridToWorld(n.row, n.col));
            n = n.parent;
        }
        Collections.reverse(raw);

        // ---- line-of-sight smoothing ----
        return smoothPath(field, raw);
    }

    // ---- internal helpers ----

    /** Diagonal distance heuristic (admissible for 8-direction movement). */
    private double heuristic(int r1, int c1, int r2, int c2) {
        int dr = Math.abs(r1 - r2);
        int dc = Math.abs(c1 - c2);
        if (dr > dc) {
            return dr + SQRT2_MINUS_1 * dc;
        } else {
            return dc + SQRT2_MINUS_1 * dr;
        }
    }

    /** Convert grid (row, col) to world-coordinate Pose2d. */
    private Pose2d gridToWorld(int row, int col) {
        double x = col - FieldMap.HALF_FIELD + 0.5;
        double y = row - FieldMap.HALF_FIELD + 0.5;
        return new Pose2d(x, y, 0);
    }

    /**
     * Line-of-sight path smoothing: removes intermediate waypoints
     * when a direct line between two waypoints is obstacle-free.
     */
    private List<Pose2d> smoothPath(FieldMap field, List<Pose2d> raw) {
        if (raw.size() <= 2) return raw;

        List<Pose2d> smooth = new ArrayList<>();
        smooth.add(raw.get(0));

        int anchor = 0;
        while (anchor < raw.size() - 1) {
            int furthest = anchor + 1;
            // Walk backwards from the end to find the furthest visible point.
            for (int i = raw.size() - 1; i > anchor; i--) {
                if (hasLineOfSight(field, raw.get(anchor), raw.get(i))) {
                    furthest = i;
                    break;
                }
            }
            smooth.add(raw.get(furthest));
            anchor = furthest;
        }
        return smooth;
    }

    /**
     * Bresenham-based line-of-sight check between two world-coordinate points.
     */
    private boolean hasLineOfSight(FieldMap field, Pose2d a, Pose2d b) {
        int c0 = FieldMap.worldToGridX(a.x);
        int r0 = FieldMap.worldToGridY(a.y);
        int c1 = FieldMap.worldToGridX(b.x);
        int r1 = FieldMap.worldToGridY(b.y);

        int dc = Math.abs(c1 - c0);
        int dr = Math.abs(r1 - r0);
        int sc = c0 < c1 ? 1 : -1;
        int sr = r0 < r1 ? 1 : -1;
        int err = dc - dr;

        int c = c0, r = r0;
        while (true) {
            if (r != r0 || c != c0) {   // skip start cell
                if (field.isObstacleCell(c, r)) return false;
            }
            if (r == r1 && c == c1) break;
            int e2 = 2 * err;
            if (e2 > -dr) { err -= dr; c += sc; }
            if (e2 <  dc) { err += dc; r += sr; }
        }
        return true;
    }

    // ---- A* node ----

    private static class Node implements Comparable<Node> {
        final int row, col;
        final double g, h;
        final Node parent;

        Node(int row, int col, double g, double h, Node parent) {
            this.row = row; this.col = col;
            this.g = g; this.h = h; this.parent = parent;
        }

        double f() { return g + h; }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.f(), other.f());
        }
    }
}
