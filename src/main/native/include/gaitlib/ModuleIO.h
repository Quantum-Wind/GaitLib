// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

namespace gaitlib {

/** Everything the module reads from its hardware each loop. */
struct ModuleIOInputs {
  bool driveConnected = false;
  bool steerConnected = false;
  bool cvtConnected = false;

  /**
   * Wheel-side (CVT output) angle. This must come from a sensor after the
   * CVT: with a variable ratio, motor rotations do not map to distance.
   */
  units::radian_t wheelPosition = 0_rad;
  units::radians_per_second_t wheelVelocity = 0_rad_per_s;

  units::radians_per_second_t driveMotorVelocity = 0_rad_per_s;
  units::volt_t driveAppliedVoltage = 0_V;
  units::ampere_t driveCurrent = 0_A;

  /** Absolute module heading. */
  frc::Rotation2d steerAngle;
  units::radians_per_second_t steerVelocity = 0_rad_per_s;

  /** Measured CVT ratio (input rotations per output rotation). */
  double cvtRatio = 1.0;
};

/**
 * Hardware abstraction for one module.
 *
 * GaitLib does not depend on any motor-controller vendor library. The robot
 * project implements this interface with whatever hardware the module uses
 * (TalonFX, SPARK, servo for the CVT, etc.). ModuleIOSim is the
 * physics-simulation implementation.
 */
class ModuleIO {
 public:
  virtual ~ModuleIO() = default;

  /** Refreshes the inputs from hardware. Called once per loop. */
  virtual void UpdateInputs(ModuleIOInputs& inputs) = 0;

  /** Closed-loop drive motor velocity (motor side, before any reduction). */
  virtual void SetDriveMotorVelocity(units::radians_per_second_t velocity) = 0;

  /** Closed-loop module heading. */
  virtual void SetSteerAngle(const frc::Rotation2d& angle) = 0;

  /** CVT ratio setpoint (input rotations per output rotation). */
  virtual void SetCvtRatio(double ratio) = 0;

  /** Neutral all outputs. */
  virtual void Stop() = 0;
};

}  // namespace gaitlib
