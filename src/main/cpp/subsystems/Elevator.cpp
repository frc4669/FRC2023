// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() = default;

// This method will be called once per scheduler run
void Elevator::Periodic() {}

units::inch_t Elevator::GetDistance() {
    return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ManipulatorConstants::kElevatorInchesPerTick);
}

frc2::CommandPtr Elevator::SetDistanceCommand(units::inch_t distance) {
    return this->Run(
        [this, distance] {
            double current = GetDistance().value();
            double output = m_heightController.Calculate(current, distance.value());
            m_mainMotor.Set(output);
        }
    ).Until([this, distance] { return units::math::abs(distance - GetDistance()) < ManipulatorConstants::kElevatorSetpointThreshold; });
}