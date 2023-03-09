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
  void SetDistance(WPI_TalonFX* motor, units::inch_t distance, double kElevatorInchesPerTick); 
  bool Home(WPI_TalonFX* motor); 

  //units::inch_t GetDistanceVertical();

  frc2::CommandPtr SetDistanceCommandVertical( units::inch_t distance );

  frc2::CommandPtr ZeroVertical(); 
 
  frc2::CommandPtr MoveVertical(int direction);

  void ConfigMotor(WPI_TalonFX* motor);

  frc2::CommandPtr DefaultControlCommand(std::function<double()> magnitude);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  WPI_TalonFX m_verticalMotor { VerticalElevatorConstants::kElevatorID };

  bool m_verticalZeroed = false; 
};
