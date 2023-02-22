// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret.h"

Turret::Turret() {
  m_rotationMotor.ConfigMotionCruiseVelocity(20000);
  m_rotationMotor.ConfigMotionAcceleration(7000);

  m_rotationMotor.SetNeutralMode(NeutralMode::Brake);
  m_rotationMotor.SetSafetyEnabled(false);

  m_rotationMotor.Config_kP(0, TurretConstants::kTurretP);
  m_rotationMotor.Config_kI(0, TurretConstants::kTurretI);
  m_rotationMotor.Config_kD(0, TurretConstants::kTurretD);
  m_rotationMotor.Config_kF(0, TurretConstants::kTurretV.value());
};

// This method will be called once per scheduler run
void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Turret Angle", GetAngle().value());
}

frc2::CommandPtr Turret::ZeroCommand() {
  return RunOnce([this] {
    Zero();
  });
}

void Turret::SetSpeed(double output) {
  m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, output);
}

void Turret::Zero() {
  m_rotationMotor.GetSensorCollection().SetQuadraturePosition(0);
}

units::degree_t Turret::GetAngle() {
  return units::degree_t(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() * TurretConstants::kTurretDegreesPerTick);
}

frc2::CommandPtr Turret::SetAngleCommand(units::degree_t angle) {
  return Run([this] {
    // no logic here for some reason...
  }).BeforeStarting([this, angle] {
    double desiredAngle = angle.value() / TurretConstants::kTurretDegreesPerTick;
    frc::SmartDashboard::PutNumber("Desired Angle (ticks)", desiredAngle);
    m_rotationMotor.Set(TalonSRXControlMode::MotionMagic, desiredAngle);
  }).Until([this] {
    double velocity = m_rotationMotor.GetSensorCollection().GetQuadratureVelocity();

    return std::abs(velocity) < 10;
  }); // .WithTimeout(5_s);

  /*return Run([this] {
    double currentAngle = m_rotationMotor.GetSensorCollection().GetQuadraturePosition();
    frc::SmartDashboard::PutNumber("Current Error (degrees)", (m_rotationController.GetSetpoint() - currentAngle) * TurretConstants::kTurretDegreesPerTick);
    m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, -m_rotationController.Calculate(currentAngle));
  })
    .Until([this] { return m_rotationController.AtSetpoint(); })
    .BeforeStarting([this, angle] {
      m_rotationController.Reset();
      m_rotationController.SetSetpoint(angle.value() / TurretConstants::kTurretDegreesPerTick);
      frc::SmartDashboard::PutNumber("Desired Angle (ticks)", m_rotationController.GetSetpoint());
    });*/
}

frc2::CommandPtr Turret::DefaultControlCommand(std::function<double()> magnitude) {
  return Run([this, magnitude = std::move(magnitude)] {
    SetSpeed(magnitude());
  });
}