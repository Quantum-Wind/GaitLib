// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>

#include <gtest/gtest.h>
#include <units/math.h>

#include "gaitlib/CvtKinematics.h"
#include "gaitlib/CvtSwerveModule.h"
#include "gaitlib/sim/ModuleIOSim.h"

using namespace gaitlib;

namespace {

CvtSwerveModule MakeSimModule(const CvtSwerveModuleConfig& config = {}) {
  return CvtSwerveModule{config, std::make_unique<sim::ModuleIOSim>(config)};
}

void RunLoops(CvtSwerveModule& module, const frc::SwerveModuleState& state,
              int loops) {
  for (int i = 0; i < loops; ++i) {
    module.Periodic();
    module.SetDesiredState(state);
  }
  module.Periodic();
}

}  // namespace

TEST(CvtSwerveModuleTest, ReachesCommandedState) {
  auto module = MakeSimModule();
  frc::SwerveModuleState target{2_mps, frc::Rotation2d{45_deg}};

  RunLoops(module, target, 150);

  auto state = module.GetState();
  EXPECT_NEAR(state.speed.value(), 2.0, 0.05);
  EXPECT_NEAR(state.angle.Degrees().value(), 45.0, 1.0);
  EXPECT_GT(module.GetPosition().distance.value(), 0.0);
}

TEST(CvtSwerveModuleTest, ShiftsTowardSpeedEndWhenFast) {
  CvtSwerveModuleConfig config;
  auto module = MakeSimModule(config);

  RunLoops(module, {0.5_mps, frc::Rotation2d{}}, 100);
  double slowRatio = module.GetCvtRatio();

  RunLoops(module, {config.maxWheelSpeed, frc::Rotation2d{}}, 100);
  double fastRatio = module.GetCvtRatio();

  EXPECT_NEAR(slowRatio, SelectCvtRatio(0.5_mps, config), 0.01);
  EXPECT_LT(fastRatio, slowRatio);
}

TEST(CvtSwerveModuleTest, OptimizesInsteadOfSpinningAround) {
  auto module = MakeSimModule();
  RunLoops(module, {0_mps, frc::Rotation2d{}}, 50);

  // Asking for 180 degrees should drive backwards at ~0 degrees instead.
  RunLoops(module, {1_mps, frc::Rotation2d{180_deg}}, 150);

  auto state = module.GetState();
  EXPECT_NEAR(state.angle.Degrees().value(), 0.0, 1.0);
  EXPECT_NEAR(state.speed.value(), -1.0, 0.05);
}

TEST(CvtSwerveModuleTest, ClampsToMaxWheelSpeed) {
  CvtSwerveModuleConfig config;
  config.maxWheelSpeed = 3_mps;
  auto module = MakeSimModule(config);

  RunLoops(module, {10_mps, frc::Rotation2d{}}, 150);

  EXPECT_NEAR(module.GetState().speed.value(), 3.0, 0.05);
}
