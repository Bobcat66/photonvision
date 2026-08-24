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
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>

#include "photon/gtsam/FieldLayout.h"
#include "photon/gtsam/Localizer.h"
#include "gtsam_jni_utils.h"

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

  jint* tagIDsPtr = env->GetIntArrayElements(tagIDs, nullptr);
  jdouble* tagPosesPtr = env->GetDoubleArrayElements(tagPoses, nullptr);
  jdouble* tagCornersPtr = env->GetDoubleArrayElements(tagCorners, nullptr);

  std::vector<wpi::apriltag::AprilTag> tags;
  // TODO: Verify tag poses length is 6 * tagIDsLength
  for (jsize i = 0; i < tagIDsLength; ++i) {
    jint tagID = tagIDsPtr[i];
    jdouble* tagPosePtr = tagPosesPtr + i * 6;

    wpi::math::Pose3d pose{
        wpi::math::Translation3d(wpi::units::meter_t(tagPosePtr[0]),
                                 wpi::units::meter_t(tagPosePtr[1]),
                                 wpi::units::meter_t(tagPosePtr[2])),
        wpi::math::Rotation3d(wpi::units::radian_t(tagPosePtr[3]),
                              wpi::units::radian_t(tagPosePtr[4]),
                              wpi::units::radian_t(tagPosePtr[5]))};

    tags.emplace_back(tagID, jdoublePtrToPose3d(tagPosePtr));
  }

  wpi::apriltag::AprilTagFieldLayout field(
      tags, wpi::units::meter_t(fieldWidth), wpi::units::meter_t(fieldLength));

  std::vector<wpi::math::Translation3d> verts;
  for (jsize i = 0; i < tagCornersLength; i += 3) {
    jdouble* cornerPtr = tagCornersPtr + i;
    verts.emplace_back(wpi::units::meter_t(cornerPtr[0]),
                       wpi::units::meter_t(cornerPtr[1]),
                       wpi::units::meter_t(cornerPtr[2]));
  }

  photon::TargetModel tagModel(verts);

  photon::pvgtsam::FieldLayout fieldLayout =
      photon::pvgtsam::FieldLayout(field, tagModel);
  photon::pvgtsam::Localizer* localizer_handle =
      new photon::pvgtsam::Localizer(fieldLayout);

  env->ReleaseIntArrayElements(tagIDs, tagIDsPtr, 0);
  env->ReleaseDoubleArrayElements(tagPoses, tagPosesPtr, 0);
  env->ReleaseDoubleArrayElements(tagCorners, tagCornersPtr, 0);

  return reinterpret_cast<jlong>(localizer_handle);
}

/*
 * Class: org_photonvision_jni_GTSAMLocalizer
 * Method: destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_org_photonvision_jni_GTSAMLocalizer_destroy
  (JNIEnv*, jclass, jlong localizer_handle)
{
  delete reinterpret_cast<photon::pvgtsam::Localizer*>(localizer_handle);
}

/*
 * Class:     org_photonvision_jni_GTSAMLocalizer
 * Method:    Reset
 * Signature: (J[DJJ)V
 */
JNIEXPORT void JNICALL
Java_org_photonvision_jni_GTSAMLocalizer_Reset
  (JNIEnv* env, jclass, jlong localizer_handle, jdoubleArray wTrArray, jlong noise_handle, jlong timeUs)
{
  jint* wTrPtr = env->GetDoubleArrayElements(wTrArray, nullptr);
  reinterpret_cast<photon::pvgtsam::Localizer*>(localizer_handle)
      ->Reset(jdoublePtrToGtsamPose3(wTrPtr),
              *reinterpret_cast<gtsam::SharedNoiseModel*>(noise_handle),
              static_cast<uint64_t>(timeUs));
  env->ReleaseDoubleArrayElements(wTrArray, wTrPtr, 0);
}
}  // extern "C"

/*
 * Class:     org_photonvision_jni_GTSAMLocalizer
 * Method:    Optimize
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_org_photonvision_jni_GTSAMLocalizer_Optimize
  (JNIEnv*, jclass, jlong localizer_handle)
{
  reinterpret_cast<photon::pvgtsam::Localizer*>(localizer_handle)->Optimize();
}
