// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

class Turret : public frc2::SubsystemBase {
 public:
  Turret();
  void Periodic() override;

  units::degree_t GetAngle();
  units::degrees_per_second_t GetVelocity();
  void SetSpeed(double output);

  frc2::CommandPtr HomeCommand();
  frc2::CommandPtr SetAngleCommand(units::degree_t angle);

  void Zero();

 private:
  WPI_TalonSRX m_mainMotor { CAN::kTurretMain };

  frc2::PIDController m_controller { TurretConstants::kp, TurretConstants::ki, TurretConstants::kd };
  frc::SimpleMotorFeedforward<units::degrees> m_feedforward { TurretConstants::ks, TurretConstants::kv, TurretConstants::ka };

  bool m_isHomed = false;
};
