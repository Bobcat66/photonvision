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

#include <string>
#include <vector>

#include <org_photonvision_jni_GTSAMLocalizer.h>
#include <wpi/apriltag/AprilTag.hpp>
#include <wpi/apriltag/AprilTagFieldLayout.hpp>
#include <wpi/units/length.hpp>

#include "photon/gtsam/FieldLayout.h"
#include "photon/gtsam/Localizer.h"

extern "C" {

/*
 * Class:     org_photonvision_jni_GTSAMLocalizer
 * Method:    create
 * Signature: ([I[DDD[D)J
 */
JNIEXPORT jlong JNICALL
Java_org_photonvision_jni_GTSAMLocalizer_create
  (JNIEnv* env, jclass, jintArray tagIDs, jdoubleArray tagPoses,
   jdouble fieldWidth, jdouble fieldLength, jdoubleArray tagCorners)
{
  jsize tagIDsLength = env->GetArrayLength(tagIDs);
  jsize tagCornersLength = env->GetArrayLength(tagCorners);

  std::vector<wpi::apriltag::AprilTag> tags;
  // TODO: Verify tag poses length is 6 * tagIDsLength
  for (jsize i = 0; i < tagIDsLength; ++i) {
    jint tagID = env->GetIntArrayElements(tagIDs, nullptr)[i];
    jdouble* tagPosePtr =
        env->GetDoubleArrayElements(tagPoses, nullptr) + i * 6;

    wpi::math::Pose3d pose{
        wpi::math::Translation3d(wpi::units::meter_t(tagPosePtr[0]),
                                 wpi::units::meter_t(tagPosePtr[1]),
                                 wpi::units::meter_t(tagPosePtr[2])),
        wpi::math::Rotation3d(wpi::units::radian_t(tagPosePtr[3]),
                              wpi::units::radian_t(tagPosePtr[4]),
                              wpi::units::radian_t(tagPosePtr[5]))};

    tags.emplace_back(tagID, pose);
  }

  wpi::apriltag::AprilTagFieldLayout field(
      tags, wpi::units::meter_t(fieldWidth), wpi::units::meter_t(fieldLength));

  std::vector<wpi::math::Translation3d> verts;
  for (jsize i = 0; i < tagCornersLength; i += 3) {
    jdouble* cornerPtr = env->GetDoubleArrayElements(tagCorners, nullptr) + i;
    verts.emplace_back(wpi::units::meter_t(cornerPtr[0]),
                       wpi::units::meter_t(cornerPtr[1]),
                       wpi::units::meter_t(cornerPtr[2]));
  }

  photon::TargetModel tagModel(verts);

  photon::pvgtsam::FieldLayout fieldLayout =
      photon::pvgtsam::FieldLayout(field, tagModel);
  photon::pvgtsam::Localizer* localizer =
      new photon::pvgtsam::Localizer(fieldLayout);
  return reinterpret_cast<jlong>(localizer);
}

}  // extern "C"
