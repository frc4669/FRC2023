// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
  ConfigMotor(&m_verticalMotor);
  ConfigMotor(&m_horizontalMotor);
};

// This method will be called once per scheduler run
void Elevator::Periodic() {}

void Elevator::ConfigMotor(WPI_TalonFX* motor) {
  motor->ConfigMotionCruiseVelocity(30000);
  motor->ConfigMotionAcceleration(8000);

  motor->SelectProfileSlot(0, 0);
  motor->Config_kP(0, VerticalElevatorConstants::kElevatorP);
  motor->Config_kI(0, VerticalElevatorConstants::kElevatorI);
  motor->Config_kD(0, VerticalElevatorConstants::kElevatorD);

  motor->ConfigNominalOutputForward(0);
  motor->ConfigNominalOutputReverse(0);
  motor->ConfigPeakOutputForward(1);
  motor->ConfigPeakOutputReverse(-1);

  motor->SetNeutralMode(NeutralMode::Brake);
  motor->SetSafetyEnabled(false);
  motor->SetInverted(true);

  motor->GetSensorCollection().SetIntegratedSensorPositionToAbsolute();
  motor->GetSensorCollection().SetIntegratedSensorPosition(0);
}; 

units::inch_t Elevator::GetDistance(WPI_TalonFX* motor, double kElevatorInchesPerTick) {
  return units::inch_t(motor->GetSensorCollection().GetIntegratedSensorPosition() * kElevatorInchesPerTick);
};

void Elevator::SetDistance(WPI_TalonFX* motor, units::inch_t distance, double kElevatorInchesPerTick) {
    double ticks = distance.value() / kElevatorInchesPerTick; 
    motor->Set(TalonFXControlMode::MotionMagic, ticks); 
}; 

frc2::CommandPtr Elevator::SetDistanceCommandVertical(units::inch_t distance) {
    return this->RunOnce(
        [this, distance] {
            SetDistance(&m_verticalMotor, distance, VerticalElevatorConstants::kElevatorInchesPerTick);
        }
    );
}

frc2::CommandPtr Elevator::SetDistanceCommandHorizontal(units::inch_t distance) {
    return this->RunOnce(
        [this, distance] {
            SetDistance(&m_horizontalMotor, distance, HorizontalElevatorConstants::kElevatorInchesPerTick);
        }
    );
}

// units::inch_t Elevator::GetDistanceVertical() {
//     return units::inch_t(m_verticalMotor.GetSensorCollection().GetIntegratedSensorPosition() * VerticalElevatorConstants::kElevatorInchesPerTick);
// }

// units::inch_t Elevator::GetDistanceHorizontal() {
//     return units::inch_t(m_horizontalMotor.GetSensorCollection().GetIntegratedSensorPosition() * HorizontalElevatorConstants::kElevatorInchesPerTick);
// }

// frc2::CommandPtr Elevator::SetDistanceCommandVertical(units::inch_t distance) {
//     return this->Run(
//         [this, distance] {
//             double current = GetDistanceVertical().value();
//             double output = m_verticalController.Calculate(current, distance.value());
//             m_verticalMotor.Set(output);
//         }
//     ).Until([this, distance] { return units::math::abs(distance - GetDistanceVertical()) < VerticalElevatorConstants::kElevatorSetpointThreshold; });
// }

// frc2::CommandPtr Elevator::SetDistanceCommandHorizontal(units::inch_t distance) {
//     return this->Run(
//         [this, distance] {
//             double current = GetDistanceHorizontal().value();
//             double output = m_horizontalController.Calculate(current, distance.value());
//             m_horizontalMotor.Set(output);
//         }
//     ).Until([this, distance] { return units::math::abs(distance - GetDistanceHorizontal()) < HorizontalElevatorConstants::kElevatorSetpointThreshold; });
// }