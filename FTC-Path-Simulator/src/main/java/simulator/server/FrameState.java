package simulator.server;

import simulator.model.Pose2d;
import simulator.physics.BallProjectileEngine;
import java.util.List;

/**
 * Snapshot of simulation state for one frame, serialised over TCP to
 * the Metal renderer.
 */
public class FrameState {

    public double elapsed;
    public boolean finished;
    public String statusText;

    // robot
    public double robotX, robotY, robotHeading;

    // trail — compact float arrays [x,y,h, ...]
    public float[] trail;

    // A* path waypoints
    public float[] path;

    // ball
    public boolean ballActive;
    public boolean ballScored;
    public double ballX, ballY, ballZ;
    public float[] ballTrajectory;  // [x,y,z, x,y,z, ...]

    public FrameState() {}

    /** Fill from engine query results. */
    public void fill(double elapsed, boolean finished, String statusText,
                     Pose2d robotPose,
                     List<Pose2d> trailList,
                     List<Pose2d> pathList,
                     BallProjectileEngine ballEngine) {
        this.elapsed = elapsed;
        this.finished = finished;
        this.statusText = statusText;

        this.robotX = robotPose.x;
        this.robotY = robotPose.y;
        this.robotHeading = robotPose.heading;

        this.trail = toFloat3(trailList);
        this.path  = toFloat3(pathList);

        this.ballActive = ballEngine.isActive();
        this.ballScored = ballEngine.isScored();
        this.ballX = ballEngine.getX();
        this.ballY = ballEngine.getY();
        this.ballZ = ballEngine.getZ();

        List<double[]> traj = ballEngine.getTrajectory();
        ballTrajectory = new float[traj.size() * 3];
        int i = 0;
        for (double[] pt : traj) {
            ballTrajectory[i++] = (float) pt[0];
            ballTrajectory[i++] = (float) pt[1];
            ballTrajectory[i++] = (float) pt[2];
        }
    }

    private static float[] toFloat3(List<Pose2d> list) {
        if (list == null || list.isEmpty()) return new float[0];
        float[] arr = new float[list.size() * 3];
        int i = 0;
        for (Pose2d p : list) {
            arr[i++] = (float) p.x;
            arr[i++] = (float) p.y;
            arr[i++] = (float) p.heading;
        }
        return arr;
    }
}
