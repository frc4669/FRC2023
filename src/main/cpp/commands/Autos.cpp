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
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.1, 1_s),
      Positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Blue_Left_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
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
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.3, 1_s),
      Positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Blue_Center_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
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
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.3, 1_s),
      Positioning::ScoreHighLeftCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Blue_Right_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
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
    Positioning::CubePickupSelectCommand(claw),
    Positioning::ScoreMidCenterCommand(elevator, extension, pivot, claw),
    Positioning::DropAndStowCommand(elevator, extension, pivot, claw)
  );
}

frc2::CommandPtr autos::L3CubeRight(
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    Positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret),
    Positioning::DropAndStowCommand(elevator, extension, pivot, claw)
  );
}
frc2::CommandPtr autos::L3CubeLeft(
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    Positioning::ScoreHighLeftCommand(elevator, extension, pivot, claw, turret),
    Positioning::DropAndStowCommand(elevator, extension, pivot, claw)
  );
}

frc2::CommandPtr autos::Red_Left_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.3, 1_s),
      Positioning::ScoreHighRightCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Red_Left_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
  );
}

frc2::CommandPtr autos::Red_Center_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.3, 1_s),
      Positioning::ScoreHighLeftCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Red_Center_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
  );
}

frc2::CommandPtr autos::Red_Right_L3Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    frc2::cmd::Parallel(
      // drivetrain->MoveCommand(-0.3, 1_s),
      Positioning::ScoreHighLeftCommand(elevator, extension, pivot, claw, turret)
    ),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Red_Right_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
  );
}

frc2::CommandPtr autos::Red_Right_L2Cube_Mobility(
  Drivetrain* drivetrain,
  Elevator* elevator,
  Extension* extension,
  Pivot* pivot,
  Claw* claw,
  Turret* turret,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    Positioning::CubePickupSelectCommand(claw),
    Positioning::ScoreMidCenterCommand(elevator, extension, pivot, claw),
    frc2::cmd::Parallel(
      Positioning::DropAndStowCommand(elevator, extension, pivot, claw),
      frc2::cmd::Wait(2_s).AndThen(FollowTrajectoryCommandNew(drivetrain, field, "Red_Right_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr())
    )
  );
}

frc2::CommandPtr autos::Red_Right_Mobility(
  Drivetrain* drivetrain,
  frc::Field2d* field
) {
  return frc2::cmd::Sequence(
    FollowTrajectoryCommandNew(drivetrain, field, "Red_Right_Mobility", false, 2_mps, 1.5_mps_sq).ToPtr()
  );
}