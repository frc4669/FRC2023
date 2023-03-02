// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

RobotContainer::RobotContainer() : m_drivetrain(&m_field), m_vision(&m_field, &m_drivetrain), m_turret(&m_vision) {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  ConfigureBindings();
  ConfigureAutonomous();

  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Scheduled Autonomous Routine", &m_autoChooser);
}

void RobotContainer::ConfigureAutonomous() {
  m_autoChooser.SetDefaultOption("Default Auto", m_defaultAutoCommand.get());
  m_autoChooser.AddOption("Curve Auto", m_curveAutoCommand.get());
}

void RobotContainer::ConfigureBindings() {
  m_drivetrain.SetDefaultCommand(m_drivetrain.DefaultDriveCommand(
    [this] { return -m_driverController.GetLeftY(); },
    [this] { return -m_driverController.GetRightX() * OperatorConstants::kTurningSpeedMutiplier; }
  ));

  m_driverController.RightTrigger().OnTrue(m_drivetrain.BoostCommand(1.0));
  m_driverController.RightTrigger().OnFalse(m_drivetrain.BoostCommand(0.3));

  m_operatorController.Y().OnTrue(m_claw.ChangeActivationState());
  m_operatorController.X().OnTrue(m_claw.SelectPressure());

  m_operatorController.Start().OnTrue(m_turret.HomeCommand());
  m_operatorController.LeftBumper().OnTrue(m_turret.AlignToTarget());

  m_turret.SetDefaultCommand(m_turret.DefaultControlCommand(
    [this] { return m_operatorController.GetRightX(); }
  ));

  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretN).OnTrue(m_turret.SetAngleCommand(0_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretE).OnTrue(m_turret.SetAngleCommand(90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretW).OnTrue(m_turret.SetAngleCommand(-90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretS).OnTrue(m_turret.SetAngleCommand(180_deg));

  // m_driverController.RightBumper().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.ResetOdometry(frc::Pose2d(), frc::Rotation2d()); }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}
