// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Extension : public frc2::SubsystemBase {
 public:
  Extension();
  void Periodic() override;

  units::inch_t GetExtension();
  units::meters_per_second_t GetVelocity();

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetExtensionCommand(units::inch_t extension);
  frc2::CommandPtr SetHomedCommand();
  frc2::CommandPtr DefaultControlCommand(std::function<double()> speed);

  void SetMotor(double output); 
  void SetUpTest();
 private:
  WPI_TalonFX m_mainMotor { CAN::kExtensionMain };

  frc2::PIDController m_controller { ExtensionConstants::kp, ExtensionConstants::ki, ExtensionConstants::kd };

  bool m_isHomed = false; 
};
