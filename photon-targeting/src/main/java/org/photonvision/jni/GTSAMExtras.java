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
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.util.Num;

public class GTSAMExtras {
    public static final class NoiseModel implements AutoCloseable {
        private final long handle;
        private static final Cleaner cleaner = Cleaner.create();
        private final Cleanable cleanable;

        // This class should only be constructed inside GTSAMExtras
        private NoiseModel(long handle, Runnable cleanup) {
            this.handle = handle;
            this.cleanable = cleaner.register(this, cleanup);
        }

        @Override
        public void close() {
            cleanable.clean();
        }

        public long getHandle() {
            return handle;
        }
    }

    public static <S extends Num> NoiseModel CreateGaussianNoiseModel(Matrix<S, S> covariances) {
        var covariances_t =
                covariances
                        .transpose(); // this will change the matrix to column-major order, which is what GTSAM
        // expects. We do not mathematically transpose the matrix, this is purely
        // memory order tomfoolery
        long handle = CreateGaussianNoiseModelJNI(covariances_t.getNumRows(), covariances_t.getData());
        return new NoiseModel(handle, () -> DestroyGaussianNoiseModelJNI(handle));
    }

    // This returns a handle to a gaussian noise model, built from a covariance matrix. the covariance
    // matrix should be stored as a flattened SQUARE COLUMN-MAJOR matrix
    // the covariance matrix should be of size matsize x matsize, and the covariances array should be
    // of length matsize * matsize
    private static native long CreateGaussianNoiseModelJNI(int matsize, double[] covariances);

    private static native long DestroyGaussianNoiseModelJNI(long handle);
}
