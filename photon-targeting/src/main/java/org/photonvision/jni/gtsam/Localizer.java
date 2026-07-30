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
import org.wpilib.vision.apriltag.ApriltagFieldLayout;
import org.photonvision.estimation.TargetModel;
import org.wpilib.geometry.Transform3d;


public class GTSAMLocalizerJNI {
    private static final Cleaner cleaner = Cleaner.create();

    private final Cleanable cleanable;

    private final long handle;

    private static Runnable cleanupAction(long ptr) {
        return () -> Localizer.JNI_destroy(ptr);
    }

    public Localizer(ApriltagFieldLayout layout, TargetModel model) {
        handle = JNI_create(layout, model);
        cleanable = cleaner.register(this, cleanupAction(handle));
    }

    public long getJNIHandle() {
        return handle;
    }

    private static native long JNI_create(ApriltagFieldLayout layout, TargetModel model);
    private static native void JNI_destroy(long handle);
    private static native void JNI_optimize(long handle);
    private static native void JNI_reset(long handle, Transform3d wTr, long noisemodel_handle, long timestamp);
}
