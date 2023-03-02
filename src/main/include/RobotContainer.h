// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"
#include "subsystems/Claw.h"
#include "subsystems/Elevator.h"
#include "subsystems/Turret.h"

#include "commands/Autos.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  frc2::CommandXboxController m_driverController { OperatorConstants::kDriverController };
  frc2::CommandXboxController m_operatorController { OperatorConstants::kOperatorController };
  frc2::CommandGenericHID m_buttonBoardA { OperatorConstants::kButtonBoardA }; 
  frc2::CommandGenericHID m_buttonBoardB { OperatorConstants::kButtonBoardB }; 

  frc::Field2d m_field;

  Drivetrain m_drivetrain;
  Vision m_vision;
  Claw m_claw;
  Elevator m_elevator;
  Turret m_turret;

  frc2::CommandPtr m_curveAutoCommand { autos::TestCurveAutoCommand(&m_drivetrain, &m_field) };
  frc2::CommandPtr m_defaultAutoCommand { autos::StraightLineAutoCommand(&m_drivetrain, &m_field) };

  frc::SendableChooser<frc2::Command*> m_autoChooser;

  void ConfigureBindings();
  void ConfigureAutonomous();
};
