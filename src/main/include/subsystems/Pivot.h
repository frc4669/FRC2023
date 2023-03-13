// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc2/command/CommandPtr.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();

  frc2::CommandPtr SetPosInCommand();  // 0
  frc2::CommandPtr SetPosDownCommand(); // 90
  frc2::CommandPtr SetPosOutCommand(); // 180

  void SetAngle(units::degree_t angle);

  frc2::CommandPtr SetAngleCommand(units::degree_t angle);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX m_controlMotor = { PivotConstants::kPivotID }; 

  frc2::PIDController m_controller = { PivotConstants::kPivotP, PivotConstants::kPivotI, PivotConstants::kPivotD };
};
