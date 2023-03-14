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
    extension->SetExtensionCommand(PositioningConstants::kShelfExtensionLength),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition)
  );
}

frc2::CommandPtr positioning::GroundPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kGroundElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kGroundExtensionLength),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition)
  );
}

frc2::CommandPtr positioning::StowCommand(Elevator* elevator, Extension* extension, Pivot* pivot) {
  return frc2::cmd::Sequence(
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kStowPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kStowExtensionLength),
    elevator->SetHeightCommand(PositioningConstants::kStowElevatorHeight)
  );
}

frc2::CommandPtr positioning::ScoreLowCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(PositioningConstants::kLowTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kLowPivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kLowElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kLowExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(PositioningConstants::kMidCenterTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kMidCenterPivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kMidCenterExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(-PositioningConstants::kMidSideTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreMidRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(PositioningConstants::kMidSideTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreHighLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(-PositioningConstants::kHighSideTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  );
}

frc2::CommandPtr positioning::ScoreHighRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    StowCommand(elevator, extension, pivot),
    turret->SetAngleCommand(-PositioningConstants::kHighSideTurretAngle),
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  );
}