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

import org.photonvision.common.dataflow.structures.Packet;
import org.wpilib.math.geometry.*;
import java.lang.ref.Cleaner;
import java.lang.ref.Cleaner.Cleanable;
import java.util.function.Function;

public class JNIHandle implements AutoCloseable {
    private static final Cleaner cleaner = Cleaner.create();

    private final Cleanable cleanable;
    private final long rawHandle;
    private volatile boolean closed = false;

    public JNIHandle(long nativeHandle, Function<Long, Runnable> cleanupActionFactory) {
        this.cleanable = cleaner.register(this, cleanupActionFactory.apply(nativeHandle));
        this.rawHandle = nativeHandle;
    }

    public long get() {
        if (closed) {
            throw new IllegalStateException("JNIHandle already closed");
        }
        return rawHandle;
    }

    public boolean isClosed() {
        return closed;
    }

    @Override
    public void close() {
        closed = true;
        cleanable.clean();
    }
}