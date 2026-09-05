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

package org.photonvision.jni;

import java.lang.ref.Cleaner;
import java.lang.ref.Cleaner.Cleanable;
import org.photonvision.estimation.TargetModel;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/*
 * serialization format:
 * Pose3d: [x (m), y (m), z (m), roll (rad), pitch (rad), yaw (rad)]
 */

/**
 * A wrapper around the GTSAM localizer implemented in C++.
 *
 * <p>This class is not thread-safe. It should only be used from a single thread at a time.
 */
public class GTSAMLocalizer {
    private final long handle;
    private final Cleaner cleaner = Cleaner.create();
    private final Cleanable cleanable;

    public GTSAMLocalizer(AprilTagFieldLayout layout, TargetModel model) {
        var tags = layout.getTags();
        int[] tagIDs = new int[tags.size()];
        double[] tagPoses = new double[tags.size() * 6];
        for (int i = 0; i < tags.size(); i++) {
            tagIDs[i] = tags.get(i).ID;
            tagPoses[i * 6] = tags.get(i).pose.getX();
            tagPoses[i * 6 + 1] = tags.get(i).pose.getY();
            tagPoses[i * 6 + 2] = tags.get(i).pose.getZ();
            tagPoses[i * 6 + 3] = tags.get(i).pose.getRotation().getX();
            tagPoses[i * 6 + 4] = tags.get(i).pose.getRotation().getY();
            tagPoses[i * 6 + 5] = tags.get(i).pose.getRotation().getZ();
        }
        double[] tagCorners = new double[3 * model.vertices.size()];
        for (int i = 0; i < model.vertices.size(); i++) {
            tagCorners[i * 3] = model.vertices.get(i).getX();
            tagCorners[i * 3 + 1] = model.vertices.get(i).getY();
            tagCorners[i * 3 + 2] = model.vertices.get(i).getZ();
        }
        long ptr =
                JNI_Localizer_create(
                        tagIDs, tagPoses, layout.getFieldWidth(), layout.getFieldLength(), tagCorners);
        cleanable = cleaner.register(this, () -> JNI_Localizer_destroy(ptr));
        handle = ptr;
    }

    public void reset(Pose3d wTr, long noiseHandle, long timeUs) {
        JNI_Localizer_Reset(
                handle,
                new double[] {
                    wTr.getX(),
                    wTr.getY(),
                    wTr.getZ(),
                    wTr.getRotation().getX(),
                    wTr.getRotation().getY(),
                    wTr.getRotation().getZ()
                },
                noiseHandle,
                timeUs);
    }

    public void addOdometry(Pose3d poseDelta, long odometryNoise_handle, long timeUs) {
        JNI_Localizer_AddOdometry(
                handle,
                new double[] {
                    poseDelta.getX(),
                    poseDelta.getY(),
                    poseDelta.getZ(),
                    poseDelta.getRotation().getX(),
                    poseDelta.getRotation().getY(),
                    poseDelta.getRotation().getZ()
                },
                odometryNoise_handle,
                timeUs);
    }

    public void addTagObservation(
            long timeUs,
            int tagID,
            double[] corners,
            double[] cameraCal,
            Pose3d robotTcamera,
            long cameraNoise_handle) {
        JNI_Localizer_AddTagObservation(
                handle,
                timeUs,
                tagID,
                corners,
                cameraCal,
                new double[] {
                    robotTcamera.getX(),
                    robotTcamera.getY(),
                    robotTcamera.getZ(),
                    robotTcamera.getRotation().getX(),
                    robotTcamera.getRotation().getY(),
                    robotTcamera.getRotation().getZ()
                },
                cameraNoise_handle);
    }

    public void optimize() {
        JNI_Localizer_Optimize(handle);
    }

    public Pose3d getLatestWorldToBody() {
        double[] pose = JNI_Localizer_GetLatestWorldToBody(handle);
        return new Pose3d(pose[0], pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
    }

    public long getLastOdomTime() {
        return JNI_Localizer_GetLastOdomTime(handle);
    }

    public Vector<N6> getLatestPoseNoise() {
        double[] noise = JNI_Localizer_GetLatestPoseNoise(handle);
        return VecBuilder.fill(noise[0], noise[1], noise[2], noise[3], noise[4], noise[5]);
    }

    // Localizer JNI methods
    private static native long JNI_Localizer_create(
            int[] tagIDs, double[] tagPoses, double fieldWidth, double fieldLength, double[] tagCorners);

    private static native void JNI_Localizer_destroy(long localizer_handle);

    private static native void JNI_Localizer_Reset(
            long localizer_handle, double[] wTr, long odometryNoise_handle, long timeUs);

    private static native void JNI_Localizer_AddOdometry(
            long localizer_handle, double[] poseDelta, long odometryNoise_handle, long timeUs);

    private static native void JNI_Localizer_AddTagObservation(
            long localizer_handle,
            long timeUs,
            int tagID,
            double[] corners,
            double[] cameraCal,
            double[] robotTcamera,
            long cameraNoise_handle);

    private static native void JNI_Localizer_Optimize(long localizer);

    private static native double[] JNI_Localizer_GetLatestWorldToBody(long localizer_handle);

    private static native long JNI_Localizer_GetLastOdomTime(long localizer_handle);

    private static native double[] JNI_Localizer_GetLatestPoseNoise(long localizer_handle);
}
