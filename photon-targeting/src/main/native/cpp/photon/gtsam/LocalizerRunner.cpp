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

#include "photon/gtsam/LocalizerRunner.h"

namespace photon::pvgtsam {

void LocalizerRunner::Accept(DataSubmission submission) {
  std::lock_guard<std::mutex> lock(mtx);
  submissionQueue.push(std::move(submission));
}

void LocalizerRunner::Process(const DataSubmission& submission) {
  switch (submission.type) {
    case DataSubmissionType::Reset: {
      const ResetData& resetData = std::get<ResetData>(submission.data);
      localizer.Reset(resetData.wTr, resetData.noise, resetData.timeUs);
      break;
    }
    case DataSubmissionType::Odometry: {
      const OdometryObservation& odom =
          std::get<OdometryObservation>(submission.data);
      localizer.AddOdometry(odom);
      break;
    }
    case DataSubmissionType::TagObservation: {
      const CameraVisionObservation& tagDetection =
          std::get<CameraVisionObservation>(submission.data);
      localizer.AddTagObservation(tagDetection);
      break;
    }
  }
}

void LocalizerRunner::Update() {
  std::lock_guard<std::mutex> lock(pose_mtx);
  poseHistory.push_back(GtsamToFrcPose3d(localizer.GetLatestWorldToBody()));
  latestOdomTime = localizer.GetLastOdomTime();
  currStateIdx = localizer.GetCurrStateIdx();
}

void LocalizerRunner::Run() {
  while (!stop_requested) {
    // Process all pending submissions
    {
      std::lock_guard<std::mutex> lock(mtx);
      while (!submissionQueue.empty()) {
        DataSubmission submission = std::move(submissionQueue.front());
        submissionQueue.pop();
        Process(submission);
      }
    }
    localizer.Optimize();
    Update();
  }
}

const wpi::math::Pose3d LocalizerRunner::GetLatestWorldToBody() {
  // TODO: add check for whether or not poseHistory is populated
  std::lock_guard<std::mutex> lock(pose_mtx);
  return poseHistory.back();
}

std::vector<wpi::math::Pose3d> LocalizerRunner::GetPoseHistory() const {
  std::lock_guard<std::mutex> lock(pose_mtx);
  return poseHistory;
}
}  // namespace photon::pvgtsam