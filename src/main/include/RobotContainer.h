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
#include "subsystems/Pivot.h"
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
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{ 0 };
  frc2::CommandXboxController m_operatorController{ 1 };

  frc2::CommandGenericHID m_pickupBoard{ 2 }; 
  frc2::CommandGenericHID m_scoringBoard{ 3 }; 

  frc::Field2d m_field;

  // The robot's subsystems are defined here...

  Drivetrain m_drivetrain;
  Vision m_vision;
  Claw m_claw;
  Elevator m_elevator;
  Turret m_turret;
  Pivot m_pivot;

  frc2::CommandPtr m_curveAutoCommand { autos::TestCurveAutoCommand(&m_drivetrain, &m_field) };
  frc2::CommandPtr m_defaultAutoCommand { autos::StraightLineAutoCommand(&m_drivetrain, &m_field) };

  frc::SendableChooser<frc2::Command*> m_autoChooser;

  void ConfigureBindings();
  void ConfigureAutonomous();
};
