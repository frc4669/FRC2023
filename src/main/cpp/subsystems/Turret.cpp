// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret.h"

Turret::Turret(Vision* vision) : m_vision(vision) {
  m_rotationMotor.ConfigMotionCruiseVelocity(20000);
  m_rotationMotor.ConfigMotionAcceleration(7000);

  m_rotationMotor.SetNeutralMode(NeutralMode::Brake);
  m_rotationMotor.SetSafetyEnabled(false);
  m_rotationMotor.SetInverted(true);

  m_rotationMotor.Config_kP(0, TurretConstants::kTurretP);
  m_rotationMotor.Config_kI(0, TurretConstants::kTurretI);
  m_rotationMotor.Config_kD(0, TurretConstants::kTurretD);
  m_rotationMotor.Config_kF(0, TurretConstants::kTurretV.value());

  m_rotationMotor.OverrideLimitSwitchesEnable(false);
  m_rotationMotor.ConfigForwardSoftLimitEnable(false);
  m_rotationMotor.ConfigReverseSoftLimitEnable(false);

  m_rotationController.SetTolerance(400);
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

frc2::CommandPtr Turret::HomeCommand() {
  return Run([this] {
    SetSpeed(-0.1);
  })
    .Until([this] { return m_rotationMotor.IsFwdLimitSwitchClosed(); })
    .AndThen(ZeroCommand())
    .AndThen(SetAngleCommand(180_deg))
    .AndThen(ZeroCommand())
    .AndThen([this] { m_isHomed = true; });
}

void Turret::SetSpeed(double output) {
  double position = m_rotationMotor.GetSensorCollection().GetQuadraturePosition();
  if (m_isHomed == true
    && ((output < 0 && position <= TurretConstants::kTurretRevThreshold)
    || (output > 0 && position >= TurretConstants::kTurretFwdThreshold))) {
      m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, 0);
      return;
  }
  m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, output);
}

void Turret::Zero() {
  m_rotationMotor.GetSensorCollection().SetQuadraturePosition(0);
}

units::degree_t Turret::GetAngle() {
  return units::degree_t(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() * TurretConstants::kTurretDegreesPerTick);
}

frc2::CommandPtr Turret::SetAngleCommand(units::degree_t angle) {
  /*return Run([this] {
    // no logic here for some reason...
  }).BeforeStarting([this, angle] {
    double desiredAngle = angle.value() / TurretConstants::kTurretDegreesPerTick;
    frc::SmartDashboard::PutNumber("Desired Angle (ticks)", desiredAngle);
    m_rotationMotor.Set(TalonSRXControlMode::MotionMagic, desiredAngle);
  }).Until([this] {
    double velocity = m_rotationMotor.GetSensorCollection().GetQuadratureVelocity();

    return std::abs(velocity) < 10;
  });*/ // .WithTimeout(5_s);

  return Run([this] {
    double currentAngle = m_rotationMotor.GetSensorCollection().GetQuadraturePosition();
    frc::SmartDashboard::PutNumber("Current Error", (m_rotationController.GetSetpoint() - currentAngle));
    m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, m_rotationController.Calculate(currentAngle));
  })
    .Until([this] { return /*std::abs(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() - m_rotationController.GetSetpoint()) < 500;*/ m_rotationController.AtSetpoint(); })
    .BeforeStarting([this, angle] {
      m_rotationController.Reset();
      m_rotationController.SetSetpoint(angle.value() / TurretConstants::kTurretDegreesPerTick);
      frc::SmartDashboard::PutNumber("Desired Angle (ticks)", m_rotationController.GetSetpoint());
    });
}

frc2::CommandPtr Turret::DefaultControlCommand(std::function<double()> magnitude) {
  return Run([this, magnitude = std::move(magnitude)] {
    SetSpeed(magnitude());
  });
}

frc2::CommandPtr Turret::AlignToTarget() {
  Vision::ObjDetectResults results = m_vision->ObjDetectGetResults();

  units::degree_t currentAngle = GetAngle();

  if (results.hasTargets) return this->SetAngleCommand(currentAngle + results.targets[0].yaw);
  else return this->SetAngleCommand(currentAngle); 
}