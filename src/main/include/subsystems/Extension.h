// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Extension : public frc2::SubsystemBase {
 public:
  Extension();
  void Periodic() override;

  units::meter_t GetExtension();
  units::meters_per_second_t GetVelocity();

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetExtensionCommand(units::meter_t extension);
  frc2::CommandPtr SetHomedCommand();
  frc2::CommandPtr DefaultControlCommand(std::function<double()> speed);

  void SetHomed();

 private:
  WPI_TalonFX m_mainMotor { CAN::kExtensionMain };

  frc::TrapezoidProfile<units::meters>::Constraints m_constraints { ExtensionConstants::kMaxVelocity, ExtensionConstants::kMaxAccel };
  frc::ProfiledPIDController<units::meters> m_controller { ExtensionConstants::kp, ExtensionConstants::ki, ExtensionConstants::kd, m_constraints };

  bool m_isHomed = false; 
};
