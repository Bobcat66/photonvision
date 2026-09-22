/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Claude slop wheeeeeeeeeee
package jni;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.photonvision.estimation.TargetModel;
import org.photonvision.jni.GTSAMExtras;
import org.photonvision.jni.GTSAMLocalizer;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

class GTSAMLocalizerTest {
    // =====================================================================
    // Adapters: the pieces below depend on code not shown in GTSAMLocalizer.
    // Adjust them to match your project before running.
    // =====================================================================

    /** TODO: name of the native library, or replace loadNatives() with your project's loader. */
    private static final String NATIVE_LIB = "photongtsamjni";

    /**
     * Pose3 noise in GTSAM's tangent-space order: ROTATION FIRST, then translation. If GTSAMExtras
     * reorders into GTSAM order for you, swap the argument order here.
     */
    private static GTSAMExtras.NoiseModel poseNoise(
            double rx, double ry, double rz, double tx, double ty, double tz) {
        return GTSAMExtras.CreateDiagonalNoiseModel(VecBuilder.fill(rx, ry, rz, tx, ty, tz));
    }

    /** Assumed: one 2D noise model applied per projected corner (pixels). */
    private static GTSAMExtras.NoiseModel pixelNoise(double sigmaPx) {
        return GTSAMExtras.CreateDiagonalNoiseModel(VecBuilder.fill(sigmaPx, sigmaPx));
    }

    // Assumed cameraCal layout: GTSAM Cal3_S2 vector form [fx, fy, skew, cx, cy].
    private static final double FX = 900.0;
    private static final double FY = 900.0;
    private static final double CX = 640.0;
    private static final double CY = 400.0;

    private static double[] cameraCal() {
        return new double[] {FX, FY, 0.0, CX, CY};
    }

    // =====================================================================
    // Fixtures
    // =====================================================================

    private static final long T0 = 1_000_000L; // microseconds
    private static final long DT = 20_000L; // 50 Hz odometry

    private static final TargetModel MODEL = TargetModel.kAprilTag36h11;

    // Two vertical tags facing -X (toward the robot), at camera height.
    private static final Pose3d TAG_1 = new Pose3d(5.0, 3.7, 0.5, new Rotation3d(0, 0, Math.PI));
    private static final Pose3d TAG_2 = new Pose3d(5.0, 4.3, 0.5, new Rotation3d(0, 0, Math.PI));

    // Forward-facing camera, WPILib convention (x forward, y left, z up).
    private static final Pose3d ROBOT_TO_CAMERA = new Pose3d(0.2, 0.0, 0.5, new Rotation3d());

    private static AprilTagFieldLayout layout() {
        return new AprilTagFieldLayout(
                List.of(new AprilTag(1, TAG_1), new AprilTag(2, TAG_2)), 16.5, 8.1);
    }

    private static GTSAMLocalizer newLocalizer() {
        return new GTSAMLocalizer(layout(), MODEL);
    }

    private static GTSAMExtras.NoiseModel tightPrior() {
        return poseNoise(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4);
    }

    private static GTSAMExtras.NoiseModel odomNoise() {
        return poseNoise(0.01, 0.01, 0.01, 0.02, 0.02, 0.02);
    }

    @BeforeAll
    static void loadNatives() {
        boolean loaded;
        try {
            System.loadLibrary(NATIVE_LIB);
            loaded = true;
        } catch (UnsatisfiedLinkError e) {
            loaded = false;
        }
        assumeTrue(loaded, "GTSAM JNI library not available; skipping");
    }

    // =====================================================================
    // Construction
    // =====================================================================

    @Test
    void constructsWithTagLayout() {
        assertDoesNotThrow(() -> newLocalizer());
    }

    @Test
    void constructsAndRunsWithEmptyLayout() {
        var empty = new AprilTagFieldLayout(List.of(), 16.5, 8.1);
        var localizer = new GTSAMLocalizer(empty, MODEL);
        localizer.reset(new Pose3d(), tightPrior().getHandle(), T0);
        localizer.optimize();
        assertPoseNear(new Pose3d(), localizer.getLatestWorldToBody(), 1e-6, 1e-6);
    }

    // =====================================================================
    // Pose serialization round trip (Java -> C++ -> Java)
    // =====================================================================

    static Stream<Pose3d> resetPoses() {
        return Stream.of(
                new Pose3d(),
                new Pose3d(1.0, 2.0, 0.0, new Rotation3d(0, 0, Math.PI / 2)),
                new Pose3d(-3.5, 7.25, 0.4, new Rotation3d(0.1, -0.2, 2.5)),
                new Pose3d(4.0, 1.0, 0.0, new Rotation3d(0, 0, Math.PI)), // yaw at the wrap point
                new Pose3d(0.0, 0.0, 1.0, new Rotation3d(0.3, Math.PI / 2 - 1e-6, -1.0))); // gimbal lock
    }

    @ParameterizedTest
    @MethodSource("resetPoses")
    void resetThenReadReturnsResetPose(Pose3d pose) {
        var localizer = newLocalizer();
        localizer.reset(pose, tightPrior().getHandle(), T0);
        localizer.optimize();
        assertPoseNear(pose, localizer.getLatestWorldToBody(), 1e-6, 1e-6);
    }

    // =====================================================================
    // Odometry
    // =====================================================================

    @Test
    void lastOdomTimeTracksMostRecentOdometry() {
        var localizer = newLocalizer();
        localizer.reset(new Pose3d(), tightPrior().getHandle(), T0);
        var delta = new Pose3d(0.1, 0, 0, new Rotation3d());

        for (int i = 1; i <= 5; i++) {
            long t = T0 + i * DT;
            localizer.addOdometry(delta, odomNoise(), t);
            assertEquals(t, localizer.getLastOdomTime());
        }
    }

    @Test
    void planarOdometryDriveForwardTurnDriveForward() {
        var localizer = newLocalizer();
        localizer.reset(new Pose3d(), tightPrior().getHandle(), T0);
        long t = T0;

        // 1 m forward in 10 steps
        for (int i = 0; i < 10; i++) {
            localizer.addOdometry(new Pose3d(0.1, 0, 0, new Rotation3d()), odomNoise(), t += DT);
        }
        // Turn 90 degrees left in place
        localizer.addOdometry(
                new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2)), odomNoise(), t += DT);
        // 1 m forward in the body frame, which is now +Y in the world
        for (int i = 0; i < 10; i++) {
            localizer.addOdometry(new Pose3d(0.1, 0, 0, new Rotation3d()), odomNoise(), t += DT);
        }
        localizer.optimize();

        var expected = new Pose3d(1.0, 1.0, 0.0, new Rotation3d(0, 0, Math.PI / 2));
        assertPoseNear(expected, localizer.getLatestWorldToBody(), 1e-4, 1e-4);
    }

    /**
     * Deltas with roll, pitch, and yaw. The reset round-trip test can't catch an Euler-convention
     * mismatch between Java and C++ if both conversions are wrong symmetrically; composing rotations
     * on the C++ side and comparing against WPILib's transformBy does.
     */
    @Test
    void odometryCompositionMatchesWpilibIn3d() {
        var localizer = newLocalizer();
        var start = new Pose3d(2.0, 3.0, 0.1, new Rotation3d(0.05, -0.1, 0.7));
        localizer.reset(start, tightPrior().getHandle(), T0);

        var delta = new Pose3d(0.1, 0.02, 0.01, new Rotation3d(0.02, -0.03, 0.05));
        var deltaTf = new Transform3d(delta.getTranslation(), delta.getRotation());
        Pose3d expected = start;
        long t = T0;
        for (int i = 0; i < 30; i++) {
            localizer.addOdometry(delta, odomNoise(), t += DT);
            expected = expected.transformBy(deltaTf);
        }
        localizer.optimize();

        assertPoseNear(expected, localizer.getLatestWorldToBody(), 1e-4, 1e-4);
    }

    // =====================================================================
    // Uncertainty
    // =====================================================================

    /**
     * With only a prior, the marginal equals the prior. Distinct sigmas per axis pin down the output
     * order: the Java wrapper documents [x, y, z, roll, pitch, yaw], but GTSAM's Pose3 marginal
     * covariance is rotation-first, so the C++ side must reorder.
     */
    @Test
    void stdDevsMatchPriorInTranslationThenRotationOrder() {
        var localizer = newLocalizer();
        localizer.reset(new Pose3d(), poseNoise(0.1, 0.2, 0.3, 0.01, 0.02, 0.03).getHandle(), T0);
        localizer.optimize();

        Vector<N6> std = localizer.getPoseComponentStdDevs();
        assertEquals(0.01, std.get(0), 1e-5, "x");
        assertEquals(0.02, std.get(1), 1e-5, "y");
        assertEquals(0.03, std.get(2), 1e-5, "z");
        assertEquals(0.1, std.get(3), 1e-4, "roll");
        assertEquals(0.2, std.get(4), 1e-4, "pitch");
        assertEquals(0.3, std.get(5), 1e-4, "yaw");
    }

    @Test
    void uncertaintyGrowsWithOdometryOnly() {
        var localizer = newLocalizer();
        localizer.reset(new Pose3d(), tightPrior().getHandle(), T0);
        localizer.optimize();
        Vector<N6> before = localizer.getPoseComponentStdDevs();

        long t = T0;
        for (int i = 0; i < 50; i++) {
            localizer.addOdometry(new Pose3d(0.1, 0, 0, new Rotation3d()), odomNoise(), t += DT);
        }
        localizer.optimize();
        Vector<N6> after = localizer.getPoseComponentStdDevs();

        assertTrue(after.get(0) > before.get(0), "x uncertainty should grow");
        assertTrue(after.get(1) > before.get(1), "y uncertainty should grow");
        assertTrue(after.get(5) > before.get(5), "yaw uncertainty should grow");
    }

    // =====================================================================
    // Tag observations
    // =====================================================================

    @Test
    void tagObservationsCorrectPoseAndReduceUncertainty() {
        var truth = new Pose3d(3.0, 4.0, 0.0, new Rotation3d());
        var guess = new Pose3d(3.25, 3.8, 0.0, new Rotation3d(0, 0, 0.1));

        var localizer = newLocalizer();
        localizer.reset(guess, poseNoise(0.5, 0.5, 0.5, 1.0, 1.0, 1.0).getHandle(), T0);
        localizer.optimize();
        Vector<N6> before = localizer.getPoseComponentStdDevs();

        localizer.addTagObservation(
                T0, 1, projectTag(truth, TAG_1), cameraCal(), ROBOT_TO_CAMERA, pixelNoise(1.0));
        localizer.addTagObservation(
                T0, 2, projectTag(truth, TAG_2), cameraCal(), ROBOT_TO_CAMERA, pixelNoise(1.0));
        localizer.optimize();

        assertPoseNear(truth, localizer.getLatestWorldToBody(), 0.01, 0.005);

        Vector<N6> after = localizer.getPoseComponentStdDevs();
        assertTrue(after.get(0) < before.get(0), "x uncertainty should shrink");
        assertTrue(after.get(1) < before.get(1), "y uncertainty should shrink");
        assertTrue(after.get(5) < before.get(5), "yaw uncertainty should shrink");
    }

    @Test
    void observationOfUnknownTagIsIgnored() {
        var pose = new Pose3d(3.0, 4.0, 0.0, new Rotation3d());
        var localizer = newLocalizer();
        localizer.reset(pose, tightPrior().getHandle(), T0);

        // Tag 99 is not in the layout; the corners are deliberately inconsistent with it.
        localizer.addTagObservation(
                T0,
                99,
                projectTag(pose.plus(new Transform3d(0.5, 0, 0, new Rotation3d())), TAG_1),
                cameraCal(),
                ROBOT_TO_CAMERA,
                pixelNoise(1.0));
        localizer.optimize();

        assertPoseNear(pose, localizer.getLatestWorldToBody(), 1e-6, 1e-6);
    }

    // =====================================================================
    // Helpers
    // =====================================================================

    /**
     * Projects a tag's corners into the camera with an ideal pinhole model. Corner order follows
     * MODEL's vertices, the same order passed to the native side in the constructor. Output layout
     * (assumed): [u0, v0, u1, v1, ...].
     */
    private static double[] projectTag(Pose3d robotPose, Pose3d tagPose) {
        Pose3d camPose =
                robotPose.transformBy(
                        new Transform3d(ROBOT_TO_CAMERA.getTranslation(), ROBOT_TO_CAMERA.getRotation()));
        List<Translation3d> vertices = MODEL.getFieldVertices(tagPose);
        double[] pixels = new double[vertices.size() * 2];

        for (int i = 0; i < vertices.size(); i++) {
            // Field point expressed in the camera frame (x forward, y left, z up)
            Translation3d p =
                    vertices.get(i).minus(camPose.getTranslation()).rotateBy(camPose.getRotation().inverse());
            assertTrue(p.getX() > 0, "test setup error: tag corner is behind the camera");

            // WPILib -> OpenCV optical frame: u grows right (-y), v grows down (-z)
            pixels[2 * i] = CX - FX * p.getY() / p.getX();
            pixels[2 * i + 1] = CY - FY * p.getZ() / p.getX();
        }
        return pixels;
    }

    private static void assertPoseNear(
            Pose3d expected, Pose3d actual, double translationTol, double rotationTol) {
        double dt = expected.getTranslation().getDistance(actual.getTranslation());
        double dr = expected.getRotation().relativeTo(actual.getRotation()).getAngle();
        assertTrue(
                dt <= translationTol,
                () ->
                        String.format("translation off by %.6f m: expected %s, got %s", dt, expected, actual));
        assertTrue(
                dr <= rotationTol,
                () -> String.format("rotation off by %.6f rad: expected %s, got %s", dr, expected, actual));
    }
}
