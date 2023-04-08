// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
  m_mainMotor.ConfigMotionAcceleration(5000);
  m_mainMotor.ConfigMotionCruiseVelocity(10000);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(true);

  m_mainMotor.OverrideLimitSwitchesEnable(true); // Reverse limit = up (inverted)

  m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(ElevatorConstants::kHomePosition.value() / ElevatorConstants::kInchesPerTick);
}

void Elevator::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Height (in)", units::inch_t(GetHeight()).value());
}

units::meter_t Elevator::GetHeight() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ElevatorConstants::kInchesPerTick);
}

units::meters_per_second_t Elevator::GetVelocity() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * ElevatorConstants::kInchesPerTick) / 1_s;
}

frc2::CommandPtr Elevator::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.3); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(ElevatorConstants::kLimitSwitchSeparation.value() / ElevatorConstants::kInchesPerTick);
      m_isHomed = true; 
    })
    .AndThen(SetHeightCommand(20_in))
    .AndThen([this] { 
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0);
    });
}

frc2::CommandPtr Elevator::SetHeightCommand(units::meter_t height) {
  return frc2::cmd::Either(
    Run([this, height] {
      units::meter_t currentHeight = GetHeight();
      double output = -m_mainController.Calculate(currentHeight);
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, output);
    })
    .BeforeStarting([this, height] {
      m_mainController.Reset(GetHeight(), GetVelocity());
      m_mainController.SetGoal(height);
    })
    .Until([this, height] {
      return
        units::math::abs(GetVelocity()) < ElevatorConstants::kVelocityThreshold
        && units::math::abs(GetHeight() - height) < ElevatorConstants::kPositionThreshold;
    }).AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}

frc2::CommandPtr Elevator::SetToSafePivotHeightCommand() {
  return frc2::cmd::Either(
    SetHeightCommand(PositioningConstants::kSafePivotHeight),
    RunOnce([] {}),
    [this] { return GetHeight() < PositioningConstants::kSafePivotHeight; }
  );
}

frc2::CommandPtr Elevator::SetHomedCommand() {
  return RunOnce([this] { m_isHomed = true; });
}

frc2::CommandPtr Elevator::DefaultControlCommand(std::function<double()> speed) {
  return Run([this, speed = std::move(speed)] {
    m_mainMotor.Set(TalonFXControlMode::PercentOutput, speed());
  });
}

frc2::CommandPtr Elevator::MoveCommand(double output) {
  return Run([this, output] {
    m_mainMotor.Set(TalonFXControlMode::PercentOutput, output);
  });
}

void Elevator::SetHomed() {
  m_isHomed = true;
}