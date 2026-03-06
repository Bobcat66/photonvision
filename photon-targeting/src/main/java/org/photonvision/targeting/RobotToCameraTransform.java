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

package org.photonvision.targeting;

import org.photonvision.common.dataflow.structures.PacketSerde;
import org.photonvision.struct.MultiTargetPNPResultSerde;
import org.photonvision.struct.RobotToCameraTransformSerde;
import org.photonvision.targeting.serde.PhotonStructSerializable;
import org.photonvision.targeting.proto.RobotToCameraTransformProto;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

// We need a wrapper class for photonserde, as this needs to be an optional value
public class RobotToCameraTransform 
    implements ProtobufSerializable, PhotonStructSerializable<RobotToCameraTransform> {
    public Transform3d robotToCamera;

    public RobotToCameraTransform() {
        this.robotToCamera = new Transform3d();
    }

    public RobotToCameraTransform(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    public Transform3d getTransform3d() {
        return robotToCamera;
    }

    /** RobotToCameraTransform protobuf for serialization. */
    public static final RobotToCameraTransformProto proto = new RobotToCameraTransformProto();

    /** RobotToCameraTransform PhotonStruct for serialization. */
    public static final RobotToCameraTransformSerde photonStruct = new RobotToCameraTransformSerde();

    @Override
    public PacketSerde<RobotToCameraTransform> getSerde() {
        return photonStruct;
    }

    public static Transform3d unwrap(RobotToCameraTransform transform) {
        return transform.getTransform3d();
     }
}
