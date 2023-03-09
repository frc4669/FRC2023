// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc/controller/PIDController.h>

class HorizontalExtension : public frc2::SubsystemBase {
 public:
  HorizontalExtension();

  units::inch_t GetDistance(WPI_TalonFX* motor, double kElevatorInchesPerTick);
  void SetDistance(WPI_TalonFX* motor, units::inch_t distance, double kElevatorInchesPerTick); 
  bool Home(WPI_TalonFX* motor); 

  //units::inch_t GetDistanceVertical();
  //units::inch_t GetDistanceHorizontal(); 

  frc2::CommandPtr SetDistanceCommandHorizontal( units::inch_t distance );

  frc2::CommandPtr ZeroHorizontal();
 
  frc2::CommandPtr MoveHorizontal(int direction);

  void ConfigMotor(WPI_TalonFX* motor); 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr DefaultControlCommand(std::function<double()> magnitude);


 private:
  WPI_TalonFX m_horizontalMotor { HorizontalElevatorConstants::kElevatorID};

  bool m_horizontalZeroed = false; 
};
