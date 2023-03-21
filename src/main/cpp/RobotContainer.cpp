// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>

RobotContainer::RobotContainer() : m_drivetrain(&m_field), m_vision(&m_field, &m_drivetrain) {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // ConfigureBindings();
  // ConfigureAutonomous();

  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Scheduled Autonomous Routine", &m_autoChooser);

  frc2::CommandScheduler::GetInstance().OnCommandInitialize([this] (const frc2::Command& command) {
    if (command.GetName().compare("PositioningCommandInProgress") == 0)
      frc::SmartDashboard::PutBoolean("Positioning in Progress", true);
  });

  frc2::CommandScheduler::GetInstance().OnCommandFinish([this] (const frc2::Command& command) {
    if (command.GetName().compare("PositioningCommandInProgress") == 0)
      frc::SmartDashboard::PutBoolean("Positioning in Progress", false);
  });
}

void RobotContainer::ConfigureAutonomous() {
  m_autoChooser.SetDefaultOption("Do Nothing", m_doNothingAutoCommand.get());

  m_autoChooser.AddOption("Testing Curve", m_curveAutoCommand.get());

  m_autoChooser.AddOption("Blue Left L3 Cube Mobility", m_blueLeftL3CubeMobilityAutoCommand.get());
  m_autoChooser.AddOption("Blue Center L3 Cube Mobility", m_blueCenterL3CubeMobilityAutoCommand.get());
  m_autoChooser.AddOption("Blue Right L3 Cube Mobility", m_blueRightL3CubeMobilityAutoCommand.get());

  m_autoChooser.AddOption("L2 Cube", m_L2CubeAutoCommand.get());
  m_autoChooser.AddOption("L3 Cube Right", m_L3CubeRightAutoCommand.get());
  m_autoChooser.AddOption("L3 Cube Left", m_L3CubeLeftAutoCommand.get());

  m_autoChooser.AddOption("Red Left L3 Cube Mobility", m_redLeftL3CubeMobilityAutoCommand.get());
  m_autoChooser.AddOption("Red Center L3 Cube Mobility", m_redCenterL3CubeMobilityAutoCommand.get());
  m_autoChooser.AddOption("Red Right L3 Cube Mobility", m_redRightL3CubeMobilityAutoCommand.get());

  m_autoChooser.AddOption("Red Right L2 Cube Mobility", m_redRightL2CubeMobilityAutoCommand.get());
  m_autoChooser.AddOption("Red Right Mobility", m_redRightMobilityAutoCommand.get());
  
}

void RobotContainer::ConfigureBindings() {
  m_drivetrain.SetDefaultCommand(m_drivetrain.DefaultDriveCommand(
    [this] { return -m_driverController.GetLeftY(); },
    [this] { return -m_driverController.GetRightX() * OperatorConstants::kTurningSpeedMutiplier; },
    [this] { return m_elevator.GetHeight().value(); }
  ));

  m_extension.SetDefaultCommand(m_extension.DefaultControlCommand(
    [this] { return m_operatorController.GetRightX(); }
  ));

  m_elevator.SetDefaultCommand(m_elevator.DefaultControlCommand(
    [this] { return m_operatorController.GetLeftY(); }
  ));

  m_driverController.RightTrigger().OnTrue(m_drivetrain.BoostCommand(1.0));
  m_driverController.RightTrigger().OnFalse(m_drivetrain.BoostCommand(0.3));
  m_driverController.LeftTrigger().OnTrue(m_drivetrain.BoostCommand(0.141));
  m_driverController.LeftTrigger().OnFalse(m_drivetrain.BoostCommand(0.3));

  m_operatorController.Back().OnTrue(frc2::cmd::Parallel(
    m_turret.SetHomedCommand(),
    m_elevator.SetHomedCommand(),
    m_extension.SetHomedCommand(),
    m_pivot.SetHomedCommand()
  ));

  m_operatorController.Start().OnTrue(frc2::cmd::Sequence(
    m_claw.ToggleActivationStateCommand(ClawConstants::kClosePosition),
    m_elevator.HomeCommand(),
    frc2::cmd::Parallel(m_pivot.HomeCommand(), m_extension.HomeCommand()),
    m_turret.HomeCommand()
  ));

  m_operatorController.RightBumper().OnTrue(Positioning::DropCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_driverController.RightBumper().WhileTrue(m_drivetrain.AutomaticBalanceCommand());

  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretN).OnTrue(m_turret.SetAngleCommand(0_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretE).OnTrue(m_turret.SetAngleCommand(90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretW).OnTrue(m_turret.SetAngleCommand(-90_deg));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kTurretS).OnTrue(m_turret.SetAngleCommand(180_deg));

  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupCone).OnTrue(Positioning::ConePickupSelectCommand(&m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupCube).OnTrue(Positioning::CubePickupSelectCommand(&m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupShelf).OnTrue(Positioning::ShelfPickupCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kPickupGround).OnTrue(Positioning::GroundPickupCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardA.Button(OperatorConstants::ButtonBoard::kStow).OnTrue(Positioning::StowCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));

  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreLowCenter).OnTrue(Positioning::ScoreLowCenterCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidCenter).OnTrue(Positioning::ScoreMidCenterCommand(&m_elevator, &m_extension, &m_pivot, &m_claw));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidLeft).OnTrue(Positioning::ScoreMidLeftCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreMidRight).OnTrue(Positioning::ScoreMidRightCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreHighLeft).OnTrue(Positioning::ScoreHighLeftCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
  m_buttonBoardB.Button(OperatorConstants::ButtonBoard::kScoreHighRight).OnTrue(Positioning::ScoreHighRightCommand(&m_elevator, &m_extension, &m_pivot, &m_claw, &m_turret));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}


void RobotContainer::ConfigTest() {

  // clear bindings
  m_drivetrain.SetDefaultCommand((frc2::CommandPtr)nullptr);

  m_extension.SetDefaultCommand((frc2::CommandPtr)nullptr);

  m_elevator.SetDefaultCommand((frc2::CommandPtr)nullptr);

  m_driverController.RightTrigger().OnTrue((frc2::CommandPtr)nullptr);
  m_driverController.RightTrigger().OnFalse((frc2::CommandPtr)nullptr);
  m_driverController.LeftTrigger().OnTrue((frc2::CommandPtr)nullptr);
  m_driverController.LeftTrigger().OnFalse((frc2::CommandPtr)nullptr);

  m_operatorController.Back().OnTrue((frc2::CommandPtr)nullptr);

  m_operatorController.Start().OnTrue((frc2::CommandPtr)nullptr);

  m_operatorController.RightBumper().OnTrue((frc2::CommandPtr)nullptr);
  m_driverController.RightBumper().WhileTrue((frc2::CommandPtr)nullptr);

  // set button bindings again
  m_operatorController.Y().OnTrue(m_claw.SetPressure(true));
  m_operatorController.A().OnTrue(m_claw.SetPressure(false));

  m_operatorController.X().OnTrue(m_claw.SetPosition(true));
  m_operatorController.B().OnTrue(m_claw.SetPosition(false));

}

void RobotContainer::ClearTestBinding() {
  m_operatorController.Y().OnTrue((frc2::CommandPtr)nullptr);
  m_operatorController.A().OnTrue((frc2::CommandPtr)nullptr);

  m_operatorController.X().OnTrue((frc2::CommandPtr)nullptr);
  m_operatorController.B().OnTrue((frc2::CommandPtr)nullptr);
}

void RobotContainer::RunTest() {
  m_extension.SetMotor(m_operatorController.GetLeftY() * 0.3);
  m_elevator.SetMotor(m_operatorController.GetLeftX() * 0.3);
  m_turret.SetMotor(m_operatorController.GetRightX() * 0.3);
  m_pivot.SetMotor(m_operatorController.GetRightY() * 0.3);
  

}