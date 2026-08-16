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

#include <gtest/gtest.h>
#include <wpi/apriltag/AprilTagFieldLayout.hpp>
#include <wpi/apriltag/AprilTagFields.hpp>

#include "photon/estimation/TargetModel.h"
#include "photon/gtsam/Localizer.h"

using namespace gtsam;

TEST(LocalizerTest, LatencyCompensate) {
  /*
  We want:
  x in [0,-1,0]
  y in [0,0,-1]
  z in [1,0,0]
  */
  const Pose3 bodyPcamera{Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0),
                          Point3{0.5, 0, 0.5}};

  // Cal3_S2 K(90, 960, 720);
  Cal3_S2 K(1000, 1000, 0, 960 / 2, 720 / 2);

  // setup noise using fake numbers
  // Pixel noise, in u,v coordinates
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 2.0);

  // Noise on the prior factor we use to anchor the first pose.
  // TODO: If we initialize with enough measurements, we might be able to
  // delete this prior?
  Vector6 sigmas;
  sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  // odometry noise
  Vector6 odomSigma;
  odomSigma << Vector3::Constant(0.001), Vector3::Constant(0.05);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(odomSigma);

  Cal3_S2_ cal(K);

  auto reefscape2025 = wpi::apriltag::AprilTagFieldLayout::LoadField(
      wpi::apriltag::AprilTagField::k2025ReefscapeWelded);

  photon::pvgtsam::FieldLayout layout(reefscape2025, photon::kAprilTag36h11);

  auto localizer = photon::pvgtsam::Localizer(layout);

  localizer.Reset(Pose3(), posePriorNoise, 5 * 1000);
  localizer.AddOdometry(photon::pvgtsam::OdometryObservation{
      100 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(photon::pvgtsam::OdometryObservation{
      200 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(photon::pvgtsam::OdometryObservation{
      300 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(photon::pvgtsam::OdometryObservation{
      400 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.Optimize();
  auto pose = localizer.GetLatestWorldToBody();
  pose.print("Pose: ");

  photon::pvgtsam::CameraVisionObservation obs{240000,
                                            8,
                                            {
                                                {414, 166},
                                                {457, 165},
                                                {457, 122},
                                                {412, 122},
                                            },
                                            cal,
                                            Pose3(),
                                            measurementNoise};

  // add vision to within isam's history
  localizer.AddTagObservation(obs);

  localizer.Optimize();
  pose = localizer.GetLatestWorldToBody();
  localizer.Print();

  // add but don't optimize
  localizer.AddOdometry(photon::pvgtsam::OdometryObservation{
      500 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});

  obs = {460000,
         8,
         {
             {414, 166},
             {457, 165},
             {457, 122},
             {412, 122},
         },
         cal,
         Pose3(),
         measurementNoise};

  localizer.AddTagObservation(obs);
  localizer.Optimize();
  pose = localizer.GetLatestWorldToBody();
  localizer.Print();
}
