package org.photonvision.targeting.proto;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.protobuf.Protobuf;
import org.photonvision.proto.Photon.ProtobufRobotToCameraTransform;
import org.photonvision.targeting.RobotToCameraTransform;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class RobotToCameraTransformProto
        implements Protobuf<RobotToCameraTransform, ProtobufRobotToCameraTransform> {
    @Override
    public Class<RobotToCameraTransform> getTypeClass() {
        return RobotToCameraTransform.class;
    }

    @Override
    public ProtobufRobotToCameraTransform createMessage() {
        return ProtobufRobotToCameraTransform.newInstance();
    }

    @Override
    public RobotToCameraTransform unpack(ProtobufRobotToCameraTransform msg) {
        return new RobotToCameraTransform(Transform3d.proto.unpack(msg.getRobotToCamera()));
    }

    @Override
    public void pack(ProtobufRobotToCameraTransform msg, RobotToCameraTransform value) {
        Transform3d.proto.pack(msg.getMutableRobotToCamera(), value.robotToCamera);
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufRobotToCameraTransform.getDescriptor();
    }
}
