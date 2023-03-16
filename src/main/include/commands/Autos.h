#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/Field2d.h>

#include "subsystems/Drivetrain.h"

namespace autos {
  frc2::CommandPtr TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field);
  frc2::CommandPtr StraightLineAutoCommand(Drivetrain* drivetrain, frc::Field2d* field);
}