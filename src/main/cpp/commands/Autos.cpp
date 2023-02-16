#include "Constants.h"
#include "commands/Autos.h"
#include "commands/FollowTrajectoryCommand.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr autos::TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field) {
  return FollowTrajectoryCommand(drivetrain, field, "TestCurve", DriveConstants::kMaxAutoSpeed, DriveConstants::kMaxAutoAccel).ToPtr();
}