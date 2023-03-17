// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret.h"

Turret::Turret() {
  m_mainMotor.ConfigMotionCruiseVelocity(1500);
  m_mainMotor.ConfigMotionAcceleration(1500);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(true);

  m_mainMotor.Config_kP(0, 0.01);
  m_mainMotor.Config_kI(0, 0);
  m_mainMotor.Config_kD(0, 0);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.OverrideLimitSwitchesEnable(false);
  m_mainMotor.ConfigForwardSoftLimitEnable(false);
  m_mainMotor.ConfigReverseSoftLimitEnable(false);
}

void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Turret Angle", GetAngle().value());(
  frc::SmartDashboard::PutBoolean("Fwd Limit Switch Closed", m_mainMotor.IsFwdLimitSwitchClosed() || m_mainMotor.IsRevLimitSwitchClosed()));
}

units::degree_t Turret::GetAngle() {
  return units::degree_t(m_mainMotor.GetSensorCollection().GetQuadraturePosition() * TurretConstants::kDegreesPerTick);
}

units::degrees_per_second_t Turret::GetVelocity() {
  return units::degrees_per_second_t(m_mainMotor.GetSensorCollection().GetQuadratureVelocity() * 10 * TurretConstants::kDegreesPerTick);
}

void Turret::SetSpeed(double output) {
  double position = m_mainMotor.GetSensorCollection().GetQuadraturePosition();
  if (m_isHomed == true
    && ((output < 0 && position <= TurretConstants::kRevThreshold)
    || (output > 0 && position >= TurretConstants::kFwdThreshold))) {
      m_mainMotor.Set(TalonSRXControlMode::PercentOutput, 0);
      return;
  }
  m_mainMotor.Set(TalonSRXControlMode::PercentOutput, output);
}

frc2::CommandPtr Turret::HomeCommand() {
  return Run([this] { SetSpeed(-0.1); })
    .Until([this] { return m_mainMotor.IsFwdLimitSwitchClosed() || m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] { SetSpeed(0); m_mainMotor.GetSensorCollection().SetQuadraturePosition(-180 / TurretConstants::kDegreesPerTick); })
    .AndThen(SetAngleCommand(0_deg))
    .AndThen([this] { m_isHomed = true; });
}

frc2::CommandPtr Turret::SetAngleCommand(units::degree_t angle) {
  return Run([this] {
    double currentAngle = m_mainMotor.GetSensorCollection().GetQuadraturePosition();
    m_mainMotor.Set(TalonSRXControlMode::PercentOutput, m_controller.Calculate(currentAngle));
  }).Until([this, angle] {
    return
      units::math::abs(GetVelocity()) < TurretConstants::kVelocityThreshold
      && units::math::abs(GetAngle() - angle) < TurretConstants::kPositionThreshold;
  }).BeforeStarting([this, angle] {
    m_controller.Reset();
    m_controller.SetSetpoint(angle.value() / TurretConstants::kDegreesPerTick);
  });
}

void Turret::Zero() {
  m_mainMotor.GetSensorCollection().SetQuadraturePosition(0);
}