// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc/controller/PIDController.h>

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  units::inch_t GetDistance();

  frc2::CommandPtr SetDistanceCommand( units::inch_t distance );

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  WPI_TalonFX m_mainMotor { ManipulatorConstants::kElevatorID };

  frc::PIDController m_heightController { ManipulatorConstants::kElevatorP, ManipulatorConstants::kElevatorI, ManipulatorConstants::kElevatorD };
};
