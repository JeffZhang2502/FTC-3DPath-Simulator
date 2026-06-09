package simulator.model;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Mecanum wheel kinematics.
 */
class KinematicsTest {

    private final Robot robot = new Robot();

    @Test
    void pureForward_AllWheelsEqualAndPositive() {
        // Given: pure forward velocity, no sideways, no rotation
        PoseVelocity2d vel = new PoseVelocity2d(10.0, 0.0, 0.0);

        double[] wheels = robot.wheelKinematics(vel);

        // Then: all four wheels must be equal and positive
        assertEquals(4, wheels.length, "Should return exactly 4 wheel speeds");

        double expected = 10.0;
        for (int i = 0; i < wheels.length; i++) {
            assertEquals(expected, wheels[i], 1e-9,
                "Wheel " + i + " should be " + expected + " for pure forward motion");
        }
    }

    @Test
    void pureRotation_LeftAndRightOppositeDirection() {
        // Given: pure rotation, no translation
        PoseVelocity2d vel = new PoseVelocity2d(0.0, 0.0, 1.0);

        double[] wheels = robot.wheelKinematics(vel);
        // FL = 0 - 0 - R*1 = -R  (negative)
        // FR = 0 + 0 + R*1 = +R  (positive)
        // BL = 0 + 0 - R*1 = -R  (negative)
        // BR = 0 - 0 + R*1 = +R  (positive)
        double R = 0.18 + 0.18; // WIDTH + LENGTH

        assertEquals(-R, wheels[0], 1e-9, "FL should be negative (left side)");
        assertEquals(+R, wheels[1], 1e-9, "FR should be positive (right side)");
        assertEquals(-R, wheels[2], 1e-9, "BL should be negative (left side)");
        assertEquals(+R, wheels[3], 1e-9, "BR should be positive (right side)");

        // Left side (FL, BL) and right side (FR, BR) must rotate in opposite directions
        assertTrue(wheels[0] < 0 && wheels[2] < 0,
            "Left wheels should rotate backward (negative)");
        assertTrue(wheels[1] > 0 && wheels[3] > 0,
            "Right wheels should rotate forward (positive)");
    }

    @Test
    void forwardKinematics_RoundTrip_PureForward() {
        // Verify that forward + inverse kinematics are consistent
        PoseVelocity2d original = new PoseVelocity2d(10.0, 0.0, 0.0);
        double[] wheels = robot.wheelKinematics(original);
        PoseVelocity2d recovered = robot.forwardKinematics(wheels[0], wheels[1], wheels[2], wheels[3]);

        assertEquals(original.vx, recovered.vx, 1e-9, "vx round-trip");
        assertEquals(original.vy, recovered.vy, 1e-9, "vy round-trip");
        assertEquals(original.vomega, recovered.vomega, 1e-9, "vomega round-trip");
    }

    @Test
    void forwardKinematics_RoundTrip_PureRotation() {
        PoseVelocity2d original = new PoseVelocity2d(0.0, 0.0, 1.0);
        double[] wheels = robot.wheelKinematics(original);
        PoseVelocity2d recovered = robot.forwardKinematics(wheels[0], wheels[1], wheels[2], wheels[3]);

        assertEquals(original.vx, recovered.vx, 1e-9, "vx round-trip");
        assertEquals(original.vy, recovered.vy, 1e-9, "vy round-trip");
        assertEquals(original.vomega, recovered.vomega, 1e-9, "vomega round-trip");
    }
}
