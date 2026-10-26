// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>

#include "gaitlib/CvtSwerveModuleConfig.h"
#include "gaitlib/ModuleIO.h"

namespace gaitlib::sim {

struct ModuleIOSimConstants {
  frc::DCMotor driveMotor = frc::DCMotor::KrakenX60(1);
  frc::DCMotor steerMotor = frc::DCMotor::KrakenX60(1);

  /** Motor rotations per module rotation. */
  double steerReduction = 12.8;

  /** Drivetrain inertia reflected to one wheel. */
  units::kilogram_square_meter_t wheelInertia = 0.025_kg_sq_m;
  units::kilogram_square_meter_t steerInertia = 0.004_kg_sq_m;

  /** First-order time constant of the CVT ratio actuator. */
  units::second_t cvtTimeConstant = 0.15_s;

  /** Simulation step; matches the default robot loop. */
  units::second_t loopPeriod = 20_ms;

  double driveKp = 0.01;  // volts per (rad/s) of motor velocity error
  double steerKp = 8.0;   // volts per radian of heading error
};

/**
 * Physics simulation of a CVT swerve module.
 *
 * The drive motor sees the wheel inertia reflected through the current total
 * reduction (fixedReduction * cvtRatio), so a ratio shift changes how hard
 * the motor has to work. Each UpdateInputs() call advances the simulation by
 * one loopPeriod.
 */
class ModuleIOSim : public ModuleIO {
 public:
  explicit ModuleIOSim(const CvtSwerveModuleConfig& config,
                       const ModuleIOSimConstants& constants = {});

  void UpdateInputs(ModuleIOInputs& inputs) override;
  void SetDriveMotorVelocity(units::radians_per_second_t velocity) override;
  void SetSteerAngle(const frc::Rotation2d& angle) override;
  void SetCvtRatio(double ratio) override;
  void Stop() override;

 private:
  CvtSwerveModuleConfig m_config;
  ModuleIOSimConstants m_constants;

  frc::sim::DCMotorSim m_steerSim;
  frc::PIDController m_steerController;

  units::radians_per_second_t m_driveMotorVelocity = 0_rad_per_s;
  units::radian_t m_wheelPosition = 0_rad;
  units::volt_t m_driveVoltage = 0_V;
  units::volt_t m_steerVoltage = 0_V;
  double m_cvtRatio;

  bool m_closedLoop = false;
  units::radians_per_second_t m_driveSetpoint = 0_rad_per_s;
  frc::Rotation2d m_steerSetpoint;
  double m_cvtSetpoint;
};

}  // namespace gaitlib::sim
