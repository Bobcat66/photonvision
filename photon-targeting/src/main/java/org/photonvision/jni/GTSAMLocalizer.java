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

public class GTSAMLocalizer {

    public GTSAMLocalizer()

    private final JNIHandle handle;

    // Localizer JNI methods
    private static native long JNI_Localizer_create(
        int[] tagIDs,
        double[] tagPoses,
        double fieldWidth,
        double fieldLength,
        double tagWidth,
        double tagHeight
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
