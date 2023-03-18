// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();
  void Periodic() override;

  units::degree_t GetAngle();
  double GetVelocity();

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetAngleCommand(units::degree_t angle);
  frc2::CommandPtr SetHomedCommand();

 private:
  WPI_TalonFX m_mainMotor { CAN::kPivotMain };

  frc2::PIDController m_controller { PivotConstants::kp, PivotConstants::ki, PivotConstants::kd };

  bool m_isHomed = false;
};
