// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/sendable/Sendable.h>

#include "gaitlib/CvtSwerveModuleConfig.h"
#include "gaitlib/ModuleIO.h"

namespace gaitlib {

/**
 * One CVT swerve module.
 *
 * Speaks the standard WPILib swerve types, so it drops into
 * frc::SwerveDriveKinematics / frc::SwerveDrivePoseEstimator like any other
 * module. Per loop:
 *
 *   module.Periodic();                  // read sensors
 *   module.SetDesiredState(state);      // command drive, steer, and CVT
 *
 * The drive motor setpoint is computed from the *measured* CVT ratio, so the
 * wheel tracks the commanded speed while the CVT is still shifting.
 */
class CvtSwerveModule : public wpi::Sendable {
 public:
  CvtSwerveModule(const CvtSwerveModuleConfig& config,
                  std::unique_ptr<ModuleIO> io);

  /** Reads hardware inputs. Call once per loop before using the getters. */
  void Periodic();

  /** Optimizes, cosine-scales, and applies a module state. */
  void SetDesiredState(frc::SwerveModuleState desiredState);

  /** Neutral all outputs. */
  void Stop();

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;

  /** Measured CVT ratio. */
  double GetCvtRatio() const { return m_inputs.cvtRatio; }

  /** Most recently commanded CVT ratio. */
  double GetCvtRatioSetpoint() const { return m_cvtRatioSetpoint; }

  const ModuleIOInputs& GetInputs() const { return m_inputs; }
  const CvtSwerveModuleConfig& GetConfig() const { return m_config; }

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  CvtSwerveModuleConfig m_config;
  std::unique_ptr<ModuleIO> m_io;
  ModuleIOInputs m_inputs;
  double m_cvtRatioSetpoint;
};

}  // namespace gaitlib
