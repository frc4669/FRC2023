// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pivot.h"

Pivot::Pivot() { 
  m_mainMotor.ConfigMotionCruiseVelocity(7000);
  m_mainMotor.ConfigMotionAcceleration(3000);

  m_mainMotor.Config_kP(0, PivotConstants::kp);
  m_mainMotor.Config_kI(0, PivotConstants::ki);
  m_mainMotor.Config_kD(0, PivotConstants::kd);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(true);
}

void Pivot::Periodic() {}

units::degree_t Pivot::GetAngle() {
  return units::degree_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * PivotConstants::kDegreesPerTick);
}

void Pivot::SetAngle(units::degree_t angle) {
  double ticks = angle.value() / PivotConstants::kDegreesPerTick;
  m_mainMotor.Set(TalonFXControlMode::MotionMagic, ticks); 
}

frc2::CommandPtr Pivot::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
      m_isHomed = true;
    });
}

frc2::CommandPtr Pivot::SetAngleCommand(units::degree_t angle) {
  return RunOnce([this, angle] {
    SetAngle(angle);
  });
}