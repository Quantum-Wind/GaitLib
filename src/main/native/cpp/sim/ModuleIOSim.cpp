// Copyright (c) 2026 Quantum Wind
// SPDX-License-Identifier: BSD-3-Clause

#include "gaitlib/sim/ModuleIOSim.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <frc/system/plant/LinearSystemId.h>
#include <units/math.h>

#include "gaitlib/CvtKinematics.h"

namespace gaitlib::sim {

namespace {
constexpr units::volt_t kMaxVoltage = 12_V;
}  // namespace

ModuleIOSim::ModuleIOSim(const CvtSwerveModuleConfig& config,
                         const ModuleIOSimConstants& constants)
    : m_config{config},
      m_constants{constants},
      m_steerSim{frc::LinearSystemId::DCMotorSystem(constants.steerMotor,
                                                    constants.steerInertia,
                                                    constants.steerReduction),
                 constants.steerMotor},
      m_steerController{constants.steerKp, 0.0, 0.0},
      m_cvtRatio{config.maxCvtRatio},
      m_cvtSetpoint{config.maxCvtRatio} {
  m_steerController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void ModuleIOSim::UpdateInputs(ModuleIOInputs& inputs) {
  const auto dt = m_constants.loopPeriod;
  const auto& motor = m_constants.driveMotor;

  if (m_closedLoop) {
    auto feedforward = units::volt_t{m_driveSetpoint.value() / motor.Kv.value()};
    auto feedback = units::volt_t{
        m_constants.driveKp * (m_driveSetpoint - m_driveMotorVelocity).value()};
    m_driveVoltage = feedforward + feedback;

    m_steerVoltage = units::volt_t{m_steerController.Calculate(
        m_steerSim.GetAngularPosition().value(),
        m_steerSetpoint.Radians().value())};
  }
  m_driveVoltage = std::clamp(m_driveVoltage, -kMaxVoltage, kMaxVoltage);
  m_steerVoltage = std::clamp(m_steerVoltage, -kMaxVoltage, kMaxVoltage);

  // CVT actuator: first-order lag toward the setpoint.
  double alpha = 1.0 - std::exp(-(dt / m_constants.cvtTimeConstant).value());
  m_cvtRatio += alpha * (m_cvtSetpoint - m_cvtRatio);

  // Drive: DC motor driving the wheel inertia reflected through the current
  // total reduction. dω/dt = a·ω + b·V, discretized exactly over the step
  // (explicit Euler goes unstable at high reductions).
  double totalReduction = m_config.fixedReduction * m_cvtRatio;
  double reflectedInertia =
      m_constants.wheelInertia.value() / (totalReduction * totalReduction);
  double a = -motor.Kt.value() /
             (motor.R.value() * motor.Kv.value() * reflectedInertia);
  double b = motor.Kt.value() / (motor.R.value() * reflectedInertia);
  double decay = std::exp(a * dt.value());
  auto previousMotorVelocity = m_driveMotorVelocity;
  m_driveMotorVelocity = units::radians_per_second_t{
      decay * m_driveMotorVelocity.value() +
      (decay - 1.0) / a * b * m_driveVoltage.value()};
  auto current = motor.Current(m_driveMotorVelocity, m_driveVoltage);

  auto wheelVelocity =
      units::radians_per_second_t{m_driveMotorVelocity.value() / totalReduction};
  auto averageWheelVelocity = units::radians_per_second_t{
      (previousMotorVelocity + m_driveMotorVelocity).value() / 2.0 /
      totalReduction};
  m_wheelPosition += averageWheelVelocity * dt;

  m_steerSim.SetInputVoltage(m_steerVoltage);
  m_steerSim.Update(dt);

  inputs.driveConnected = true;
  inputs.steerConnected = true;
  inputs.cvtConnected = true;

  inputs.wheelPosition = m_wheelPosition;
  inputs.wheelVelocity = wheelVelocity;
  inputs.driveMotorVelocity = m_driveMotorVelocity;
  inputs.driveAppliedVoltage = m_driveVoltage;
  inputs.driveCurrent = units::math::abs(current);

  inputs.steerAngle = frc::Rotation2d{m_steerSim.GetAngularPosition()};
  inputs.steerVelocity = m_steerSim.GetAngularVelocity();

  inputs.cvtRatio = m_cvtRatio;
}

void ModuleIOSim::SetDriveMotorVelocity(units::radians_per_second_t velocity) {
  m_closedLoop = true;
  m_driveSetpoint = velocity;
}

void ModuleIOSim::SetSteerAngle(const frc::Rotation2d& angle) {
  m_closedLoop = true;
  m_steerSetpoint = angle;
}

void ModuleIOSim::SetCvtRatio(double ratio) {
  m_cvtSetpoint = ClampCvtRatio(ratio, m_config);
}

void ModuleIOSim::Stop() {
  m_closedLoop = false;
  m_driveVoltage = 0_V;
  m_steerVoltage = 0_V;
}

}  // namespace gaitlib::sim
