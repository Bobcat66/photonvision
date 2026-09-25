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

#include <map>
#include <vector>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/units/time.hpp>

#include "gtsam/slam/expressions.h"
#include "photon/gtsam/FieldLayout.h"
#include "photon/gtsam/gtsam_utils.h"

#include <mutex>
#include <queue>
#include <atomic>
#include <variant>

namespace photon::pvgtsam {

class LocalizerCore {
  using Key = gtsam::Key;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;
  using LandmarkMap = std::map<Key, SmartFactor::shared_ptr>;

 public:
  explicit LocalizerCore(const wpi::fields::Field& layout,
                     const TargetModel& tagModel)
      : LocalizerCore(FieldLayout(layout, tagModel)) {}

  explicit LocalizerCore(FieldLayout fieldLayout);

  /**
   * Add a prior factor on the world->robot pose. Not threadsafe, use SubmitReset instead when multithreading
   */
  void Reset(ResetData data);

  /**
   * Threadsafe way to reset
   */
  void SubmitReset(ResetData data);

  /**
   * Not threadsafe, use SubmitOdometry instead when multithreading
   */
  void AddOdometry(OdometryObservation odom);

  /**
   * Threadsafe way to submit odometry
   */
  void SubmitOdometry(OdometryObservation odom);

  /**
   * Not threadsafe, use SubmitTagObservation instead when multithreading
   */
  void AddTagObservation(CameraVisionObservation obs);

  void SubmitTagObservation(CameraVisionObservation obs);

  /**
   * Not threadsafe, use Step() instead when multithreading
   */
  void Optimize();
  
  /**
   * The main loop function. When multithreading, this function should be called in the worker thread
   */
  void Step();

  // inline void ExportGraph(std::ostream& os) {
  //   smootherISAM2.getFactors().saveGraph(os);
  // 
  /*
  inline void Print(const std::string_view prefix = "") {
    // fmt::println("{}", prefix); TODO: fmt doesn't work anymore for some
    // reason? I blame wpilib
    smootherISAM2.print();
    smootherISAM2.getISAM2().getFactorsUnsafe().print();
    smootherISAM2.calculateEstimate().print("Current estimate:");
  }
    */

  inline Key GetCurrStateIdx() const { return currStateIdx; }
  inline uint64_t GetLastOdomTime() const { return latestOdomTime; }

  gtsam::Pose3 GetLatestWorldToBody() const;

  gtsam::Matrix GetLatestMarginals() const;
  // standard deviations on rx ry rz tx ty tz
  gtsam::Vector6 GetPoseComponentStdDevs() const;

  std::vector<wpi::math::Pose3d> GetPoseHistory() const;

 protected:
  /**
   * If a given time is fully within the smoother history, find or interplate a
   * key for it
   */
  Key InsertIntoSmoother(Key lower, Key upper, Key newKey, double newTime,
                         gtsam::SharedNoiseModel odometryNoise);

  // Claude Slop - For testing only, remove before shipping
  Key ClaudeInsertIntoSmoother(Key lower, Key upper, Key newKey,
                               double newTime);

  Key GetOrInsertKey(Key newKey, double time);

  void Accept(DataSubmission submission);
  void Process(const DataSubmission& submission);

  // New factor graph to add to our smoother at the next call to Optimize()
  gtsam::ExpressionFactorGraph graph{};
  // New inital guesses to add to our smoother at the next call to Optimize()
  gtsam::Values currentEstimate{};
  // New state timestamps to add to our smoother at the next call to Optimize()
  gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps{};
  // Factors to delete
  gtsam::FactorIndices factorsToRemove{};
  // Log of old twists
  typedef std::map<Key, gtsam::Pose3> KeyPoseDeltaMap;
  KeyPoseDeltaMap twistsFromPreviousKey{};

  // ISAM-backed fixed-lag smoother. Will marginalize out states older then a
  // given lag.
  gtsam::IncrementalFixedLagSmoother smootherISAM2;

  // Current "tip" world->body estimate
  gtsam::Pose3 wTb_latest;
  uint64_t latestOdomTime;

  // keep track of our current state. State is encoded as X(uS since epoch).
  // the Key class uses the lower 56 bits for the index, and top 8 for symbol
  // 2^(64−8)÷10^6÷60÷60÷24÷365 = 2284 years, so as long as we use a sane epoch
  // we're good. This will only work on 64-bit machines, but oh well. big shame.
  Key currStateIdx;

  FieldLayout fieldLayout;

  mutable std::mutex data_mtx;
  mutable std::mutex isam_mtx;

  std::queue<DataSubmission> submissionQueue;
};

}  // namespace photon::pvgtsam
