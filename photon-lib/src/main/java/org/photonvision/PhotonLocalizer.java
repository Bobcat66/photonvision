/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.photonvision;

import org.photonvision.estimation.TargetModel;
import org.photonvision.jni.GTSAMConcurrentLocalizer;
import org.photonvision.jni.GTSAMExtras;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N5;
import org.wpilib.system.Notifier;

public class PhotonLocalizer {
    private GTSAMConcurrentLocalizer core;
    private GTSAMExtras.NoiseModel odomNoise;
    private Notifier notifier;

    public PhotonLocalizer(Field layout, TargetModel model) {
        core = new GTSAMConcurrentLocalizer(layout, model);
        odomNoise = GTSAMExtras.NoiseModel.Diagonal(VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
        notifier = new Notifier(() -> this.core.step());
    }

    public static final record GTSAMCamConfig(
            Transform3d robotToCamera, GTSAMExtras.Cal3S2 cameraCal, GTSAMExtras.NoiseModel pixelNoise) {
        public GTSAMCamConfig(Transform3d robotToCamera, Vector<N5> cameraCal, double sigma) {
            this(
                    robotToCamera,
                    GTSAMExtras.Cal3S2.FromVector(cameraCal),
                    GTSAMExtras.NoiseModel.Diagonal(VecBuilder.fill(sigma, sigma)));
        }

        public GTSAMCamConfig(
                Transform3d robotToCamera,
                GTSAMExtras.Cal3S2 cameraCal,
                GTSAMExtras.NoiseModel pixelNoise) {
            this.robotToCamera = robotToCamera;
            this.cameraCal = cameraCal;
            this.pixelNoise = pixelNoise;
        }
    }

    public static final record GTSAMPoseEstimate(Pose3d pose, double timestamp) {}

    public void start() {
        notifier.startPeriodic(0.02);
    }

    public void stop() {
        notifier.stop();
    }

    public void setOdomNoise(Vector<N6> noise) {
        odomNoise = GTSAMExtras.NoiseModel.Diagonal(noise);
    }
}
