#include "commands/PositionSequences.h"

frc2::CommandPtr positioning::ConePickupSelectCommand(Claw* claw) {
  return frc2::cmd::Sequence(
    claw->TogglePressureCommand(ClawConstants::kConePressure),
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition)
  );
}

frc2::CommandPtr positioning::CubePickupSelectCommand(Claw* claw) {
  return frc2::cmd::Sequence(
    claw->TogglePressureCommand(ClawConstants::kCubePressure),
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition)
  );
}

frc2::CommandPtr positioning::ShelfPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetHeightCommand(PositioningConstants::kShelfElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    extension->SetExtensionCommand(PositioningConstants::kShelfExtensionLength)
  );
}

frc2::CommandPtr positioning::GroundPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kGroundExtensionLength),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    elevator->SetHeightCommand(PositioningConstants::kGroundElevatorHeight)
  );
}

frc2::CommandPtr positioning::StowCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kStowPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kStowExtensionLength),
    elevator->SetHeightCommand(PositioningConstants::kStowElevatorHeight)
  );
}

frc2::CommandPtr positioning::ScoreLowCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(PositioningConstants::kLowTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kLowPivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kLowElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kLowExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(PositioningConstants::kMidCenterTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidCenterPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidCenterExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(PositioningConstants::kMidSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(-PositioningConstants::kMidSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreHighLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(-PositioningConstants::kHighSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreHighRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(-PositioningConstants::kHighSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  );
}

frc2::CommandPtr positioning::DropCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    // elevator->SetHeightCommand(elevator->GetHeight() - PositioningConstants::kDropDelta),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    frc2::cmd::Wait(PositioningConstants::kStowDelay),
    // elevator->SetHeightCommand(elevator->GetHeight() + PositioningConstants::kDropDelta),
    StowCommand(elevator, extension, pivot, claw)
  );
}