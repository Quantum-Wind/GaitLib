// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <functional>

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace gaitlib {

/**
 * Physical description of one CVT swerve module.
 *
 * Ratio conventions used throughout GaitLib (all are reductions, i.e. input
 * rotations per output rotation, so a bigger number means more torque and
 * less speed):
 *
 *   drive motor --[fixedReduction]--> CVT input --[cvtRatio]--> wheel
 *
 *   total reduction = fixedReduction * cvtRatio
 *
 * The default values are placeholders. Fill them in from the module's CAD.
 */
struct CvtSwerveModuleConfig {
  /** Radius of the drive wheel. */
  units::meter_t wheelRadius = 0.0508_m;

  /** Product of every fixed gear/belt stage between the drive motor and the wheel. */
  double fixedReduction = 5.0;

  /** CVT ratio at the speed (overdrive) end of its travel. */
  double minCvtRatio = 0.5;

  /** CVT ratio at the torque end of its travel. */
  double maxCvtRatio = 2.0;

  /**
   * Motor speed the default ratio policy tries to hold the drive motor at.
   * Somewhere around the motor's peak-power speed is a reasonable start.
   */
  units::radians_per_second_t targetMotorSpeed = 500_rad_per_s;

  /** Fastest the module will ever be commanded to drive. */
  units::meters_per_second_t maxWheelSpeed = 6_mps;

  /**
   * Optional override for choosing the CVT ratio from the commanded wheel
   * speed. When empty, GaitLib uses PowerBandRatio(). Results are clamped to
   * [minCvtRatio, maxCvtRatio].
   */
  std::function<double(units::meters_per_second_t)> ratioPolicy;
};

}  // namespace gaitlib
