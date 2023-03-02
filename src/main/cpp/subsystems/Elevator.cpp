// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() = default;

// This method will be called once per scheduler run
void Elevator::Periodic() {}

units::inch_t Elevator::GetDistanceVertical() {
    return units::inch_t(m_verticalMotor.GetSensorCollection().GetIntegratedSensorPosition() * VerticalElevatorConstants::kElevatorInchesPerTick);
}

units::inch_t Elevator::GetDistanceHorizontal() {
    return units::inch_t(m_horizontalMotor.GetSensorCollection().GetIntegratedSensorPosition() * HorizontalElevatorConstants::kElevatorInchesPerTick);
}

frc2::CommandPtr Elevator::SetDistanceCommandVertical(units::inch_t distance) {
    return this->Run(
        [this, distance] {
            double current = GetDistanceVertical().value();
            double output = m_verticalController.Calculate(current, distance.value());
            m_verticalMotor.Set(output);
        }
    ).Until([this, distance] { return units::math::abs(distance - GetDistanceVertical()) < VerticalElevatorConstants::kElevatorSetpointThreshold; });
}

frc2::CommandPtr Elevator::SetDistanceCommandHorizontal(units::inch_t distance) {
    return this->Run(
        [this, distance] {
            double current = GetDistanceHorizontal().value();
            double output = m_horizontalController.Calculate(current, distance.value());
            m_horizontalMotor.Set(output);
        }
    ).Until([this, distance] { return units::math::abs(distance - GetDistanceHorizontal()) < HorizontalElevatorConstants::kElevatorSetpointThreshold; });
}