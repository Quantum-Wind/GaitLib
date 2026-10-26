// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include "gaitlib/CvtSwerveModule.h"

#include <utility>

#include <units/math.h>
#include <wpi/sendable/SendableBuilder.h>

#include "gaitlib/CvtKinematics.h"

namespace gaitlib {

CvtSwerveModule::CvtSwerveModule(const CvtSwerveModuleConfig& config,
                                 std::unique_ptr<ModuleIO> io)
    : m_config{config},
      m_io{std::move(io)},
      m_cvtRatioSetpoint{config.maxCvtRatio} {}

void CvtSwerveModule::Periodic() {
  m_io->UpdateInputs(m_inputs);
}

void CvtSwerveModule::SetDesiredState(frc::SwerveModuleState desiredState) {
  desiredState.Optimize(m_inputs.steerAngle);
  desiredState.CosineScale(m_inputs.steerAngle);

  auto speed = units::math::min(units::math::abs(desiredState.speed),
                                m_config.maxWheelSpeed);
  if (desiredState.speed < 0_mps) {
    speed = -speed;
  }

  m_cvtRatioSetpoint = SelectCvtRatio(speed, m_config);
  double measuredRatio = ClampCvtRatio(m_inputs.cvtRatio, m_config);

  m_io->SetCvtRatio(m_cvtRatioSetpoint);
  m_io->SetDriveMotorVelocity(WheelToMotorSpeed(speed, measuredRatio, m_config));
  m_io->SetSteerAngle(desiredState.angle);
}

void CvtSwerveModule::Stop() {
  m_io->Stop();
}

frc::SwerveModuleState CvtSwerveModule::GetState() const {
  return {units::meters_per_second_t{m_inputs.wheelVelocity.value() *
                                     m_config.wheelRadius.value()},
          m_inputs.steerAngle};
}

frc::SwerveModulePosition CvtSwerveModule::GetPosition() const {
  return {units::meter_t{m_inputs.wheelPosition.value() *
                         m_config.wheelRadius.value()},
          m_inputs.steerAngle};
}

void CvtSwerveModule::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("CvtSwerveModule");
  builder.AddDoubleProperty(
      "Speed (mps)", [this] { return GetState().speed.value(); }, nullptr);
  builder.AddDoubleProperty(
      "Angle (deg)", [this] { return m_inputs.steerAngle.Degrees().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Distance (m)", [this] { return GetPosition().distance.value(); },
      nullptr);
  builder.AddDoubleProperty(
      "CVT Ratio", [this] { return m_inputs.cvtRatio; }, nullptr);
  builder.AddDoubleProperty(
      "CVT Ratio Setpoint", [this] { return m_cvtRatioSetpoint; }, nullptr);
  builder.AddDoubleProperty(
      "Drive Motor Velocity (rad per s)",
      [this] { return m_inputs.driveMotorVelocity.value(); }, nullptr);
  builder.AddDoubleProperty(
      "Drive Current (A)", [this] { return m_inputs.driveCurrent.value(); },
      nullptr);
}

}  // namespace gaitlib
