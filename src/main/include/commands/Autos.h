#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/Field2d.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Pivot.h"
#include "subsystems/Turret.h"

namespace autos {
  frc2::CommandPtr TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field);
  frc2::CommandPtr StraightLineAutoCommand(Drivetrain* drivetrain, frc::Field2d* field);
  frc2::CommandPtr BasicAutoCommand(Drivetrain* drivetrain, frc::Field2d* field, Elevator* elevator, Claw* claw, Pivot* pivot, Turret* turret);
}