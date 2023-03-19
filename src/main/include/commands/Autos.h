#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/Field2d.h>

#include "commands/PositionSequences.h"
#include "commands/FollowTrajectoryCommandNew.h"

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Pivot.h"
#include "subsystems/Turret.h"

#include "Constants.h"

namespace autos {
  frc2::CommandPtr DoNothingAutoCommand();
  frc2::CommandPtr TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field);

  frc2::CommandPtr Blue_Left_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Blue_Center_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Blue_Right_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  
  frc2::CommandPtr L2Cube(
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret
  );
  frc2::CommandPtr L3Cube(
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret
  );

  frc2::CommandPtr Red_Left_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Red_Center_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Red_Right_L3Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Red_Right_L2Cube_Mobility(
    Drivetrain* drivetrain,
    Elevator* elevator,
    Extension* extension,
    Pivot* pivot,
    Claw* claw,
    Turret* turret,
    frc::Field2d* field
  );
  frc2::CommandPtr Red_Right_Mobility(
    Drivetrain* drivetrain,
    frc::Field2d* field
  );
}