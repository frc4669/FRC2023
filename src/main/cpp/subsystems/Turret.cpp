// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret.h"

Turret::Turret(Vision* vision) : m_vision(vision) {
  m_rotationMotor.ConfigMotionCruiseVelocity(2000);
  m_rotationMotor.ConfigMotionAcceleration(1000);

  m_rotationMotor.SetNeutralMode(NeutralMode::Brake);
  m_rotationMotor.SetSafetyEnabled(false);
  m_rotationMotor.SetInverted(true);

  m_rotationMotor.SelectProfileSlot(0, 0);
  m_rotationMotor.Config_kP(0, 0.01);
  m_rotationMotor.Config_kI(0, 0);
  m_rotationMotor.Config_kD(0, 0);
  m_rotationMotor.Config_kF(0, 0.001);

  m_rotationMotor.ConfigNominalOutputForward(0);
  m_rotationMotor.ConfigNominalOutputReverse(0);
  m_rotationMotor.ConfigPeakOutputForward(1);
  m_rotationMotor.ConfigPeakOutputReverse(-1);

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
    SetSpeed(-0.2);
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
  return Run([this] {
    frc::SmartDashboard::PutNumber("Position (ticks)", m_rotationMotor.GetSensorCollection().GetQuadraturePosition());
  }).BeforeStarting([this, angle] {
    m_rotationMotor.Set(TalonSRXControlMode::MotionMagic, angle.value() / TurretConstants::kTurretDegreesPerTick);
    frc::SmartDashboard::PutNumber("Setpoint (ticks)", angle.value() / TurretConstants::kTurretDegreesPerTick);
  }).Until([this, angle] {
    return std::abs(m_rotationMotor.GetSensorCollection().GetQuadratureVelocity()) < 100
      && std::abs(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() - (angle.value() / TurretConstants::kTurretDegreesPerTick)) < 100;
  });

  /*return Run([this] {
    double currentAngle = m_rotationMotor.GetSensorCollection().GetQuadraturePosition();
    frc::SmartDashboard::PutNumber("Current Error", (m_rotationController.GetSetpoint() - currentAngle));
    m_rotationMotor.Set(TalonSRXControlMode::PercentOutput, m_rotationController.Calculate(currentAngle));
  })
    .Until([this] { return /*std::abs(m_rotationMotor.GetSensorCollection().GetQuadraturePosition() - m_rotationController.GetSetpoint()) < 500;th th m_rotationController.AtSetpoint(); })
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

frc2::CommandPtr Turret::AlignToTarget() {
  Vision::ObjDetectResults results = m_vision->ObjDetectGetResults();

  units::degree_t currentAngle = GetAngle();

  if (results.hasTargets) return this->SetAngleCommand(currentAngle + results.targets[0].yaw);
  else return this->SetAngleCommand(currentAngle); 
}