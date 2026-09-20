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

public class GTSAMExtras {
    // This returns a handle to a gaussian noise model, built from a covariance matrix. the covariance matrix should be stored as a flattened COLUMN-MAJOR matrix
    public static native long CreateGaussianNoiseModel(double[] covariances, boolean smart);
    public static native long DestroyGaussianNoiseModel(long handle);
}
