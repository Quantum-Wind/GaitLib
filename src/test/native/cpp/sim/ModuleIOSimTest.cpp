// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "gaitlib/sim/ModuleIOSim.h"

using namespace gaitlib;

namespace {

ModuleIOInputs Step(sim::ModuleIOSim& io, int loops) {
  ModuleIOInputs inputs;
  for (int i = 0; i < loops; ++i) {
    io.UpdateInputs(inputs);
  }
  return inputs;
}

}  // namespace

TEST(ModuleIOSimTest, StartsAtRestAtTorqueEnd) {
  CvtSwerveModuleConfig config;
  sim::ModuleIOSim io{config};

  auto inputs = Step(io, 1);

  EXPECT_TRUE(inputs.driveConnected);
  EXPECT_TRUE(inputs.steerConnected);
  EXPECT_TRUE(inputs.cvtConnected);
  EXPECT_DOUBLE_EQ(inputs.cvtRatio, config.maxCvtRatio);
  EXPECT_DOUBLE_EQ(inputs.driveMotorVelocity.value(), 0.0);
  EXPECT_DOUBLE_EQ(inputs.wheelPosition.value(), 0.0);
  EXPECT_DOUBLE_EQ(inputs.driveAppliedVoltage.value(), 0.0);
}

TEST(ModuleIOSimTest, CvtRatioLagsTowardClampedSetpoint) {
  CvtSwerveModuleConfig config;
  sim::ModuleIOSim io{config};

  io.SetCvtRatio(-5.0);
  double afterOneStep = Step(io, 1).cvtRatio;
  EXPECT_LT(afterOneStep, config.maxCvtRatio);
  EXPECT_GT(afterOneStep, config.minCvtRatio);
  EXPECT_NEAR(Step(io, 100).cvtRatio, config.minCvtRatio, 1e-4);

  io.SetCvtRatio(100.0);
  EXPECT_NEAR(Step(io, 100).cvtRatio, config.maxCvtRatio, 1e-4);
}

TEST(ModuleIOSimTest, DriveTracksVelocitySetpoint) {
  CvtSwerveModuleConfig config;
  sim::ModuleIOSim io{config};

  io.SetDriveMotorVelocity(300_rad_per_s);
  auto inputs = Step(io, 150);

  EXPECT_NEAR(inputs.driveMotorVelocity.value(), 300.0, 5.0);
  EXPECT_NEAR(inputs.wheelVelocity.value(),
              inputs.driveMotorVelocity.value() /
                  (config.fixedReduction * inputs.cvtRatio),
              1e-9);
  EXPECT_GT(inputs.wheelPosition.value(), 0.0);
  EXPECT_GE(inputs.driveCurrent.value(), 0.0);
}

TEST(ModuleIOSimTest, SteerTracksAngleSetpoint) {
  sim::ModuleIOSim io{CvtSwerveModuleConfig{}};

  io.SetSteerAngle(frc::Rotation2d{90_deg});
  auto inputs = Step(io, 150);

  EXPECT_NEAR(inputs.steerAngle.Degrees().value(), 90.0, 1.0);
}

TEST(ModuleIOSimTest, OutputVoltageIsClamped) {
  sim::ModuleIOSim io{CvtSwerveModuleConfig{}};

  io.SetDriveMotorVelocity(1e6_rad_per_s);
  EXPECT_DOUBLE_EQ(Step(io, 1).driveAppliedVoltage.value(), 12.0);

  io.SetDriveMotorVelocity(-1e6_rad_per_s);
  EXPECT_DOUBLE_EQ(Step(io, 1).driveAppliedVoltage.value(), -12.0);
}

TEST(ModuleIOSimTest, StopNeutralsOutputsUntilNextCommand) {
  sim::ModuleIOSim io{CvtSwerveModuleConfig{}};
  io.SetDriveMotorVelocity(300_rad_per_s);
  double moving = Step(io, 50).driveMotorVelocity.value();

  io.Stop();
  auto inputs = Step(io, 50);

  EXPECT_DOUBLE_EQ(inputs.driveAppliedVoltage.value(), 0.0);
  EXPECT_LT(inputs.driveMotorVelocity.value(), moving);
}
