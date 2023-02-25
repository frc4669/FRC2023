// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <ctre/Phoenix.h>
#include "Vision.h"

#include "Constants.h"

class Turret : public frc2::SubsystemBase {
 public:
  Turret(Vision* vision);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::degree_t GetAngle();

  frc2::CommandPtr ZeroCommand();
  frc2::CommandPtr HomeCommand();

  frc2::CommandPtr SetAngleCommand(units::degree_t angle);

  frc2::CommandPtr DefaultControlCommand(std::function<double()> magnitude);

  frc2::CommandPtr AlignToTarget(); 

  void SetSpeed(double output);

  void Zero();

  bool IsAtLimit();

 private:
  Vision* m_vision; 

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  WPI_TalonSRX m_rotationMotor { TurretConstants::kTurretID };

  bool m_isHomed = false;
  frc2::PIDController m_rotationController { TurretConstants::kTurretP, TurretConstants::kTurretI, TurretConstants::kTurretD };
  // frc::SimpleMotorFeedforward<units::degrees> m_rotationFF { TurretConstants::kTurretS, TurretConstants::kTurretV, TurretConstants::kTurretA };
};
