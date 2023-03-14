// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  void Periodic() override;

  units::inch_t GetHeight();
  void SetHeight(units::inch_t height);

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetHeightCommand(units::inch_t height);

 private:
  WPI_TalonFX m_mainMotor { CAN::kElevatorMain };

  bool m_isHomed = false; 
};
