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

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/units/length.hpp>

#include "photon/gtsam/gtsam_utils.h"

inline wpi::math::Pose3d jdoublePtrToPose3d(jdouble* ptr) {
  return wpi::math::Pose3d{
      wpi::math::Translation3d(wpi::units::meter_t(ptr[0]),
                               wpi::units::meter_t(ptr[1]),
                               wpi::units::meter_t(ptr[2])),
      wpi::math::Rotation3d(wpi::units::radian_t(ptr[3]),
                            wpi::units::radian_t(ptr[4]),
                            wpi::units::radian_t(ptr[5]))};
}
inline wpi::math::Transform3d jdoublePtrToTransform3d(jdouble* ptr) {
  return wpi::math::Transform3d{
      wpi::math::Translation3d(wpi::units::meter_t(ptr[0]),
                               wpi::units::meter_t(ptr[1]),
                               wpi::units::meter_t(ptr[2])),
      wpi::math::Rotation3d(wpi::units::radian_t(ptr[3]),
                            wpi::units::radian_t(ptr[4]),
                            wpi::units::radian_t(ptr[5]))};
}
inline gtsam::Pose3 jdoublePtrToGtsamPose3(jdouble* ptr) {
  return gtsam::Pose3{gtsam::Rot3::Ypr(ptr[5], ptr[4], ptr[3]),
                      gtsam::Point3(ptr[0], ptr[1], ptr[2])};
}
inline jdouble* gtsamPose3ToJdoublePtr(const gtsam::Pose3& pose) {
  jdouble* ptr = new jdouble[6];
  gtsam::Point3 t = pose.translation();
  gtsam::Rot3 r = pose.rotation();
  ptr[0] = t.x();
  ptr[1] = t.y();
  ptr[2] = t.z();
  double roll, pitch, yaw;
  r.ypr(yaw, pitch, roll);
  ptr[3] = roll;
  ptr[4] = pitch;
  ptr[5] = yaw;
  return ptr;
}
