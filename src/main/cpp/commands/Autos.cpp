#include "commands/Autos.h"

frc2::CommandPtr autos::DoNothingAutoCommand() { return frc2::cmd::RunOnce([] {}); };

frc2::CommandPtr autos::TestCurveAutoCommand(Drivetrain* drivetrain, frc::Field2d* field) {
  return FollowTrajectoryCommandNew(drivetrain, field, "TestCurve", false, 2_mps, 1.5_mps_sq).ToPtr();
}

frc2::CommandPtr autos::Blue_Left_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    positioning::CubePickupSelectCommand(claw),
    drivetrain->MoveCommand(-0.2, 1.5_s),
    positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret),
    positioning::DropCommand(elevator, extension, pivot, claw),
    FollowTrajectoryCommandNew(drivetrain, field, "Blue_Left_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr()
  );
}

frc2::CommandPtr autos::Blue_Center_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    positioning::CubePickupSelectCommand(claw),
    drivetrain->MoveCommand(-0.2, 1.5_s),
    positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret),
    positioning::DropCommand(elevator, extension, pivot, claw),
    FollowTrajectoryCommandNew(drivetrain, field, "Blue_Center_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr()
  );
}

frc2::CommandPtr autos::Blue_Right_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    positioning::CubePickupSelectCommand(claw),
    drivetrain->MoveCommand(-0.2, 1.5_s),
    positioning::ScoreHighLeftCommand(elevator, extension, pivot, claw, turret),
    positioning::DropCommand(elevator, extension, pivot, claw),
    FollowTrajectoryCommandNew(drivetrain, field, "Blue_Right_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr()
  );
}

frc2::CommandPtr autos::L2Cube(
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret
) {
  return frc2::cmd::Sequence(
    positioning::CubePickupSelectCommand(claw),
    positioning::ScoreMidCenterCommand(elevator, extension, pivot, claw, turret),
    positioning::DropCommand(elevator, extension, pivot, claw)
  );
}