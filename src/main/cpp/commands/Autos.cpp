#include "Constants.h"
#include "commands/Autos.h"
#include "commands/FollowTrajectoryCommand.h"
#include "commands/FollowTrajectoryCommandNew.h"
#include "commands/PositionSequences.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr autos::TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field) {
  return FollowTrajectoryCommandNew(drivetrain, field, "TestCurve", false).ToPtr();
}

frc2::CommandPtr autos::StraightLineAutoCommand(Drivetrain* drivetrain, frc::Field2d* field) {
  return frc2::cmd::Sequence(
    FollowTrajectoryCommandNew(drivetrain, field, "StraightLine", false).ToPtr()
    // FollowTrajectoryCommandNew(drivetrain, field, "StraightLine", true).ToPtr()
  );
}

frc2::CommandPtr autos::BasicAutoCommand(Drivetrain* drivetrain, frc::Field2d* field, Elevator* elevator, Claw* claw, Pivot* pivot, Turret* turret) {
  return frc2::cmd::Sequence();
}