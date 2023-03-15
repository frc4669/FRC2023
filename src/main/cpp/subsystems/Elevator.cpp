// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
  m_mainMotor.ConfigMotionAcceleration(8000);
  m_mainMotor.ConfigMotionCruiseVelocity(30000);

  m_mainMotor.Config_kP(0, ElevatorConstants::kp);
  m_mainMotor.Config_kI(0, ElevatorConstants::ki);
  m_mainMotor.Config_kD(0, ElevatorConstants::kd);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(false);

  m_mainMotor.OverrideLimitSwitchesEnable(true);
}

void Elevator::Periodic() {}

units::inch_t Elevator::GetHeight() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ElevatorConstants::kInchesPerTick);
}

units::meters_per_second_t Elevator::GetVelocity() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * ElevatorConstants::kInchesPerTick) / 1_s;
}

void Elevator::SetHeight(units::inch_t height) {
  double ticks = height.value() / ElevatorConstants::kInchesPerTick;
  m_mainMotor.Set(TalonFXControlMode::MotionMagic, ticks); 
}

frc2::CommandPtr Elevator::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
      m_isHomed = true; 
    });
}

frc2::CommandPtr Elevator::SetHeightCommand(units::inch_t height) {
  return frc2::cmd::Either(
    Run([] {})
      .BeforeStarting([this, height] { SetHeight(height); })
      .Until([this, height] {
        return units::math::abs(GetHeight() - height) < ElevatorConstants::kPositionThreshold
          && units::math::abs(GetVelocity()) < ElevatorConstants::kVelocityThreshold;
      }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}

frc2::CommandPtr Elevator::SetToSafePivotHeightCommand() {
  return RunOnce([this] {
    if (GetHeight() < PositioningConstants::kSafePivotHeight && m_isHomed) SetHeight(PositioningConstants::kSafePivotHeight); 
  });
}