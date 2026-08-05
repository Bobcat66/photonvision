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

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.photonvision.estimation.TargetModel;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N6;
import java.lang.ref.Cleaner;
import java.lang.ref.Cleaner.Cleanable;

public class GTSAMLocalizer {

    private final long handle;
    private final Cleaner cleaner = Cleaner.create();
    private final Cleanable cleanable;

    public GTSAMLocalizer(AprilTagFieldLayout field, TargetModel model) {
        auto tags = layout.getTags();
        int[] tagIDs = new int[tags.size()];
        double[] tagPoses = new double[tags.size() * 6];
        for (int i = 0; i < tags.size(); i++) {
            tagIDs[i] = tags[i].ID;
            tagPoses[i * 6] = tags[i].getX();
            tagPoses[i * 6 + 1] = tags[i].getY();
            tagPoses[i * 6 + 2] = tags[i].getZ();
            tagPoses[i * 6 + 3] = tags[i].getRotation().getX();
            tagPoses[i * 6 + 4] = tags[i].getRotation().getY();
            tagPoses[i * 6 + 5] = tags[i].getRotation().getZ();
        }
        double[] tagVertices = new double[3 * model.vertices.size()];
        for (int i = 0; i < model.vertices.size(); i++) {
            tagCorners[i * 3] = model.vertices[i].getX();
            tagCorners[i * 3 + 1] = model.vertices[i].getY();
            tagCorners[i * 3 + 2] = model.vertices[i].getZ();
        }
        long ptr = JNI_Localizer_create(
            tagIDs, tagPoses, layout.getFieldWidth(), layout.getFieldLength(), 
        )
        cleaner.register(this, () -> JNI_Localizer_destroy(ptr));
        handle = ptr;
    }

    // Localizer JNI methods
    private static native long JNI_Localizer_create(
        int[] tagIDs,
        double[] tagPoses,
        double fieldWidth,
        double fieldLength,
        double[] tagCorners
    );
    private static native void JNI_Localizer_destroy(long localizer_handle);
    private static native void JNI_Localizer_Reset(
        long localizer_handle,
        double[] wTr,
        double[] sigmas,
        long timeUs
    );
    private static native void JNI_Localizer_AddOdometry(
        long localizer_handle,
        double[] poseDelta,
        long odometryNoise_handle,
        long timeUs
    );
    private static native void JNI_Localizer_AddTagObservation(
        long localizer_handle,
        int timeUs,
        long tagID,
        double[] corners,
        double[] cameraCal,
        double[] robotTcamera,
        long cameraNoise_handle
    );
    private static native void JNI_Localizer_Optimize(long localizer);
    private static native double[] JNI_Localizer_GetLatestWorldToBody(long localizer_handle);
    private static native long JNI_Localizer_GetLastOdomTime(long localizer_handle);
    private static native double[] JNI_Localizer_GetLatestPoseNoise(long localizer_handle);
}
