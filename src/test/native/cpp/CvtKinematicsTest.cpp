// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "gaitlib/CvtKinematics.h"

using namespace gaitlib;

TEST(CvtKinematicsTest, MotorWheelRoundTrip) {
  CvtSwerveModuleConfig config;
  for (double ratio : {config.minCvtRatio, 1.0, config.maxCvtRatio}) {
    auto motorSpeed = WheelToMotorSpeed(3_mps, ratio, config);
    EXPECT_NEAR(MotorToWheelSpeed(motorSpeed, ratio, config).value(), 3.0,
                1e-9);
  }
}

TEST(CvtKinematicsTest, HigherRatioNeedsFasterMotor) {
  CvtSwerveModuleConfig config;
  auto low = WheelToMotorSpeed(2_mps, config.minCvtRatio, config);
  auto high = WheelToMotorSpeed(2_mps, config.maxCvtRatio, config);
  EXPECT_NEAR((high / low).value(), config.maxCvtRatio / config.minCvtRatio,
              1e-9);
}

TEST(CvtKinematicsTest, PowerBandRatioSaturates) {
  CvtSwerveModuleConfig config;
  EXPECT_DOUBLE_EQ(PowerBandRatio(0_mps, config), config.maxCvtRatio);
  EXPECT_DOUBLE_EQ(PowerBandRatio(0.01_mps, config), config.maxCvtRatio);
  EXPECT_DOUBLE_EQ(PowerBandRatio(100_mps, config), config.minCvtRatio);
}

TEST(CvtKinematicsTest, PowerBandRatioHoldsTargetMotorSpeed) {
  CvtSwerveModuleConfig config;
  // Pick a speed whose ideal ratio is inside the CVT's range.
  double midRatio = (config.minCvtRatio + config.maxCvtRatio) / 2.0;
  auto wheelSpeed = MotorToWheelSpeed(config.targetMotorSpeed, midRatio, config);

  double ratio = PowerBandRatio(wheelSpeed, config);
  EXPECT_NEAR(ratio, midRatio, 1e-9);
  EXPECT_NEAR(WheelToMotorSpeed(wheelSpeed, ratio, config).value(),
              config.targetMotorSpeed.value(), 1e-6);
}

TEST(CvtKinematicsTest, PowerBandRatioIgnoresDirection) {
  CvtSwerveModuleConfig config;
  EXPECT_DOUBLE_EQ(PowerBandRatio(-2_mps, config),
                   PowerBandRatio(2_mps, config));
}

TEST(CvtKinematicsTest, ClampCvtRatioBoundsToRange) {
  CvtSwerveModuleConfig config;
  EXPECT_DOUBLE_EQ(ClampCvtRatio(0.0, config), config.minCvtRatio);
  EXPECT_DOUBLE_EQ(ClampCvtRatio(1.0, config), 1.0);
  EXPECT_DOUBLE_EQ(ClampCvtRatio(10.0, config), config.maxCvtRatio);
}

TEST(CvtKinematicsTest, DefaultPolicyIsPowerBand) {
  CvtSwerveModuleConfig config;
  EXPECT_DOUBLE_EQ(SelectCvtRatio(3_mps, config), PowerBandRatio(3_mps, config));
}

TEST(CvtKinematicsTest, CustomPolicyIsClamped) {
  CvtSwerveModuleConfig config;
  config.ratioPolicy = [](units::meters_per_second_t) { return 1000.0; };
  EXPECT_DOUBLE_EQ(SelectCvtRatio(1_mps, config), config.maxCvtRatio);
}
