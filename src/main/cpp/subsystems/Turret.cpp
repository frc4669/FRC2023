// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret.h"

Turret::Turret() {
  m_rotationMotor.SetNeutralMode(NeutralMode::Brake);
  m_rotationMotor.SetSafetyEnabled(false);
};

// This method will be called once per scheduler run
void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Turret Angle", GetAngle().value());
}

frc2::CommandPtr Turret::HomeCommand() {
  return Run([this] {
    SetSpeed(-0.6);
  })
  .Until([this] { return IsAtLimit(); })
  .AndThen([this] { SetSpeed(0.0); Zero(); });
}

void Turret::SetSpeed(double output) {
  frc::SmartDashboard::PutNumber("Desired Turret Speed", output);
  m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, output);
}

bool Turret::IsAtLimit() {
  return m_rotationMotor.IsRevLimitSwitchClosed();
}

void Turret::Zero() {
  m_rotationMotor.GetSensorCollection().SetQuadraturePosition(0);
}

units::degree_t Turret::GetAngle() {
  return units::degree_t(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() * TurretConstants::kTurretDegreesPerTick);
}

frc2::CommandPtr Turret::DefaultControlCommand(std::function<double()> magnitude) {
  return Run([this, magnitude = std::move(magnitude)] {
    SetSpeed(magnitude());
  });
}