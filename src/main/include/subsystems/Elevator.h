// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  void Periodic() override;

  units::meter_t GetHeight();
  units::meters_per_second_t GetVelocity();

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetHeightCommand(units::meter_t height);
  frc2::CommandPtr SetToSafePivotHeightCommand();
  frc2::CommandPtr SetHomedCommand();
  frc2::CommandPtr DefaultControlCommand(std::function<double()>);
  frc2::CommandPtr MoveCommand(double output);

  void SetHomed();

 private:
  WPI_TalonFX m_mainMotor { CAN::kElevatorMain };

  frc::TrapezoidProfile<units::meters>::Constraints m_constraints { ElevatorConstants::kMaxVelocity, ElevatorConstants::kMaxAccel };
  frc::ProfiledPIDController<units::meters> m_mainController { ElevatorConstants::kp, ElevatorConstants::ki, ElevatorConstants::kd, m_constraints };
  // frc::ElevatorFeedforward<units::inches> m_feedforward { ElevatorConstants::ks, ElevatorConstants::kv, ElevatorConstants::ka, ElevatorConstants::kg };

  bool m_isHomed = false; 
};
