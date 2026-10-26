// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "gaitlib/CvtSwerveModuleConfig.h"

namespace gaitlib {

/** Clamps a CVT ratio into the module's physical range. */
double ClampCvtRatio(double ratio, const CvtSwerveModuleConfig& config);

/** Linear wheel speed produced by a drive motor speed at a given CVT ratio. */
units::meters_per_second_t MotorToWheelSpeed(
    units::radians_per_second_t motorSpeed, double cvtRatio,
    const CvtSwerveModuleConfig& config);

/** Drive motor speed needed for a linear wheel speed at a given CVT ratio. */
units::radians_per_second_t WheelToMotorSpeed(
    units::meters_per_second_t wheelSpeed, double cvtRatio,
    const CvtSwerveModuleConfig& config);

/**
 * Default ratio policy: pick the ratio that keeps the drive motor at
 * config.targetMotorSpeed for the requested wheel speed. Slow commands
 * saturate at the torque end (maxCvtRatio), fast ones at the speed end
 * (minCvtRatio).
 */
double PowerBandRatio(units::meters_per_second_t wheelSpeed,
                      const CvtSwerveModuleConfig& config);

/** Applies config.ratioPolicy (or PowerBandRatio) and clamps the result. */
double SelectCvtRatio(units::meters_per_second_t wheelSpeed,
                      const CvtSwerveModuleConfig& config);

}  // namespace gaitlib
