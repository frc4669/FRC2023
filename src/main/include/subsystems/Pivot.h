// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedforward.h>

#include "Constants.h"

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();
  void Periodic() override;

  units::degree_t GetAngle();
  units::degrees_per_second_t GetVelocity();

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetAngleCommand(units::degree_t angle);
  frc2::CommandPtr SetHomedCommand();

 private:
  WPI_TalonFX m_mainMotor { CAN::kPivotMain };

  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints { PivotConstants::kMaxVelocity, PivotConstants::kMaxAccel };
  frc::ProfiledPIDController<units::degrees> m_controller { PivotConstants::kp, PivotConstants::ki, PivotConstants::kd, m_constraints };
  frc::ArmFeedforward m_feedforward { PivotConstants::ks, PivotConstants::kg, PivotConstants::kv, PivotConstants::ka };

  // WPILib: ADD STATE TO INLINE COMMANDS

  bool m_isHomed = false;
};
