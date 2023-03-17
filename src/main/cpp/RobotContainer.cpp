// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

RobotContainer::RobotContainer() : m_drivetrain(&m_field), m_vision(&m_field, &m_drivetrain) {
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
    [this] { return -m_driverController.GetRightX() * OperatorConstants::kTurningSpeedMutiplier; },
    [this] { return m_elevator.GetHeight().value(); }
  ));

  m_driverController.RightTrigger().OnTrue(m_drivetrain.BoostCommand(1.0));
  m_driverController.RightTrigger().OnFalse(m_drivetrain.BoostCommand(0.3));

  m_operatorController.X().OnTrue(m_turret.HomeCommand());

  m_operatorController.Start().OnTrue(frc2::cmd::Sequence(
    m_claw.ToggleActivationStateCommand(ClawConstants::kClosePosition),
    m_elevator.HomeCommand(),
    frc2::cmd::Parallel(m_pivot.HomeCommand(), m_extension.HomeCommand())
  ));

  m_operatorController.RightBumper().OnTrue(m_claw.ToggleActivationStateCommand());

  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretN).OnTrue(m_turret.SetAngleCommand(0_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretE).OnTrue(m_turret.SetAngleCommand(90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretW).OnTrue(m_turret.SetAngleCommand(-90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretS).OnTrue(m_turret.SetAngleCommand(180_deg));

  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupCone).OnTrue(positioning::ConePickupSelectCommand(&m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupCube).OnTrue(positioning::CubePickupSelectCommand(&m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupShelf).OnTrue(positioning::ShelfPickupCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupGround).OnTrue(positioning::GroundPickupCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kStow).OnTrue(positioning::StowCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));

  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreLowCenter).OnTrue(positioning::ScoreLowCenterCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidCenter).OnTrue(positioning::ScoreMidCenterCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidLeft).OnTrue(positioning::ScoreMidLeftCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidRight).OnTrue(positioning::ScoreMidRightCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreHighLeft).OnTrue(positioning::ScoreHighLeftCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreHighRight).OnTrue(positioning::ScoreHighRightCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}
