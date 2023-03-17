// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();
  void Periodic() override;

  units::degree_t GetAngle();
  double GetVelocity();
  void SetAngle(units::degree_t angle);

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetAngleCommand(units::degree_t angle);

 private:
  WPI_TalonFX m_mainMotor { CAN::kPivotMain };

  bool m_isHomed = false;
};
