// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
  m_mainMotor.ConfigMotionAcceleration(5000);
  m_mainMotor.ConfigMotionCruiseVelocity(10000);

  // m_mainMotor.Config_kP(0, ElevatorConstants::kp);
  // m_mainMotor.Config_kI(0, ElevatorConstants::ki);
  // m_mainMotor.Config_kD(0, ElevatorConstants::kd);
  // m_mainMotor.Config_kF(0, 0.0001);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(true);

  m_mainMotor.OverrideLimitSwitchesEnable(true); // Reverse limit = up (inverted)

  frc::SmartDashboard::PutData(&m_mainController);
}

void Elevator::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Height (in)", GetHeight().value());
}

units::inch_t Elevator::GetHeight() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ElevatorConstants::kInchesPerTick);
}

units::meters_per_second_t Elevator::GetVelocity() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * ElevatorConstants::kInchesPerTick) / 1_s;
}

// void Elevator::SetHeight(units::inch_t height) {
//   double ticks = height.value() / ElevatorConstants::kInchesPerTick;
//   frc::SmartDashboard::PutNumber("TTicks elev", ticks);
//   if(m_isHomed) m_mainMotor.Set(TalonFXControlMode::MotionMagic, ticks); 
// }

frc2::CommandPtr Elevator::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(ElevatorConstants::kLimitSwitchSeparation.value() / ElevatorConstants::kInchesPerTick);
      m_isHomed = true; 
    })
    // .AndThen(Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0.2); }).WithTimeout(4_s))
    .AndThen(SetHeightCommand(20_in))
    .AndThen([this] { 
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0);
    });
}

// frc2::CommandPtr Elevator::SetHeightCommand(units::inch_t height) {
//   return frc2::cmd::Either(
//     Run([] {})
//       .BeforeStarting([this, height] { SetHeight(height); })
//       .Until([this, height] {
//         return units::math::abs(GetHeight() - height) < ElevatorConstants::kPositionThreshold
//           && units::math::abs(GetVelocity()) < ElevatorConstants::kVelocityThreshold;
//       }),
//     RunOnce([] {}), [this] { return m_isHomed; }
//   );
// }

frc2::CommandPtr Elevator::SetHeightCommand(units::inch_t height) {
  return frc2::cmd::Either(
    Run([this, height] {
      double currentHeight = GetHeight().value();
      double output = -m_mainController.Calculate(currentHeight);
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, output);
    })
    .BeforeStarting([this, height] {
      m_mainController.Reset();
      m_mainController.SetSetpoint(height.value());
    })
    .Until([this, height] {
      return
        units::math::abs(GetVelocity()) < ElevatorConstants::kVelocityThreshold
        && units::math::abs(GetHeight() - height) < ElevatorConstants::kPositionThreshold;
    }).AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }),
    /*Run([] {}).BeforeStarting([this, height] {
      units::inch_t currentHeight = GetHeight();
      if (currentHeight < height) m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.3);
      else m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0.3);
    }).Until([this, height] { return units::math::abs(height - GetHeight()) < 5_in; })
      .AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }),*/
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