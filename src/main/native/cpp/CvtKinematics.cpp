// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include "gaitlib/CvtKinematics.h"

#include <algorithm>

#include <units/math.h>

namespace gaitlib {

double ClampCvtRatio(double ratio, const CvtSwerveModuleConfig& config) {
  return std::clamp(ratio, config.minCvtRatio, config.maxCvtRatio);
}

units::meters_per_second_t MotorToWheelSpeed(
    units::radians_per_second_t motorSpeed, double cvtRatio,
    const CvtSwerveModuleConfig& config) {
  double wheelRadPerSec =
      motorSpeed.value() / (config.fixedReduction * cvtRatio);
  return units::meters_per_second_t{wheelRadPerSec *
                                    config.wheelRadius.value()};
}

units::radians_per_second_t WheelToMotorSpeed(
    units::meters_per_second_t wheelSpeed, double cvtRatio,
    const CvtSwerveModuleConfig& config) {
  double wheelRadPerSec = wheelSpeed.value() / config.wheelRadius.value();
  return units::radians_per_second_t{wheelRadPerSec * config.fixedReduction *
                                     cvtRatio};
}

double PowerBandRatio(units::meters_per_second_t wheelSpeed,
                      const CvtSwerveModuleConfig& config) {
  // Motor speed the wheel speed would need with the CVT at 1:1.
  auto motorSpeedAtUnity = WheelToMotorSpeed(units::math::abs(wheelSpeed), 1.0,
                                             config);
  if (motorSpeedAtUnity.value() <= 0.0) {
    return config.maxCvtRatio;
  }
  return ClampCvtRatio(
      config.targetMotorSpeed.value() / motorSpeedAtUnity.value(), config);
}

double SelectCvtRatio(units::meters_per_second_t wheelSpeed,
                      const CvtSwerveModuleConfig& config) {
  double ratio = config.ratioPolicy ? config.ratioPolicy(wheelSpeed)
                                    : PowerBandRatio(wheelSpeed, config);
  return ClampCvtRatio(ratio, config);
}

}  // namespace gaitlib
