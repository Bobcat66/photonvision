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

#include "Localizer.h"
#include <wpi/system/Notifier.hpp>
#include <mutex>
#include <queue>
#include <atomic>
#include <variant>

namespace photon::pvgtsam {

// Represents data submitted to the localizer from other threads
enum class DataSubmissionType {
  Reset,
  Odometry,
  TagObservation
};

struct ResetData {
  gtsam::Pose3 wTr;
  gtsam::SharedNoiseModel noise;
  uint64_t timeUs;
};

struct DataSubmission {
    DataSubmissionType type;
    std::variant<ResetData, OdometryObservation, CameraVisionObservation> data;
};

// Runs a localizer in a separate thread, and allows for adding observations from other threads
class LocalizerRunner {
public:
    explicit LocalizerRunner(Localizer localizer) 
        : localizer(std::move(localizer))
        , notifier([this] { Run(); }) {}

  void Start() {
    // TODO: Test if this delay is necessary, or if we can just start the thread immediately
    notifier.StartSingle(wpi::units::second_t(0.01));
  }
  void Stop() {
    stop_requested = true;
  }

  void Reset(ResetData resetData);

  void AddOdometry(OdometryObservation odom);

  void AddTagObservation(CameraVisionObservation tagDetection);

  inline gtsam::Key GetCurrStateIdx() const { return currStateIdx; }
  inline uint64_t GetLastOdomTime() const { return latestOdomTime; }

  const wpi::math::Pose3d GetLatestWorldToBody();

  // gtsam::Matrix GetLatestMarginals() const;
  // standard deviations on rx ry rz tx ty tz
  // gtsam::Vector6 GetPoseComponentStdDevs() const;

  std::vector<wpi::math::Pose3d> GetPoseHistory() const;
private:
  void Run();
  void Accept(DataSubmission submission);
  void Process(const DataSubmission& submission);
  void Update();
  mutable std::mutex mtx;
  mutable std::mutex pose_mtx;
  Localizer localizer;
  wpi::Notifier notifier;
  std::atomic<uint64_t> latestOdomTime{0};
  std::atomic<gtsam::Key> currStateIdx{0};
  std::atomic<bool> stop_requested{false};
  std::vector<wpi::math::Pose3d> poseHistory;
  std::queue<DataSubmission> submissionQueue;
};
}