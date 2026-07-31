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

#pragma once

#include "photon/gtsam/impl/Localizer.h"
#include "photon/targeting/PhotonTrackedTarget.h"

namespace photon::gtsam {
class GTSAMPoseObserver{
 public:
  GTSAMPoseEstimator(photon::gtsam::FieldLayout fieldLayout);

  void Reset(wpi::geometry::Pose3d wTr, gtsam::SharedNoiseModel noise, uint64_t timeUs);

  void AddOdometry(
    const gtsam::Pose3& poseDelta,
    const gtsam::SharedNoiseModel& odometryNoise, 
    uint64_t timeUs
  );

  void AddTagObservation(
    uint64_t timeUs,
    const PhotonTrackedTarget& target,
    const wpi::geometry::Transform3d& robotTcamera,
  );

  gtsam::Pose3 GetLatestWorldToBody() const;

 private:
  pvgtsam_impl::Localizer localizer;
};
} // namespace photon::gtsam
