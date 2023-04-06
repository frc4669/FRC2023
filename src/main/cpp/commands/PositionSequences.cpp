#include "commands/PositionSequences.h"
 
frc2::CommandPtr Positioning::ConePickupSelectCommand(Claw* claw) {
  return frc2::cmd::Sequence(
    claw->TogglePressureCommand(ClawConstants::kConePressure),
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::CubePickupSelectCommand(Claw* claw) {
  return frc2::cmd::Sequence(
    claw->TogglePressureCommand(ClawConstants::kCubePressure),
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ShelfPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetHeightCommand(PositioningConstants::kShelfElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    extension->SetExtensionCommand(PositioningConstants::kShelfExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::GroundPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kGroundExtensionLength),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    elevator->SetHeightCommand(PositioningConstants::kGroundElevatorHeight)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::StowCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    claw->ToggleActivationStateCommand(ClawConstants::kClosePosition),
    pivot->SetAngleCommand(PositioningConstants::kClearPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kStowExtensionLength),
    elevator->SetToSafePivotHeightCommand(),
    elevator->SetHeightCommand(PositioningConstants::kStowElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kStowPivotAngle)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreLowCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetToSafePivotHeightCommand(),
    pivot->SetAngleCommand(PositioningConstants::kLowPivotAngle),
    elevator->SetHeightCommand(PositioningConstants::kLowElevatorHeight),
    extension->SetExtensionCommand(PositioningConstants::kLowExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreMidCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidCenterPivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidCenterExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreMidLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(PositioningConstants::kMidSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreMidRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(-PositioningConstants::kMidSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kMidElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kMidSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kMidSideExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreHighLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(PositioningConstants::kHighSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ScoreHighRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret) {
  return frc2::cmd::Sequence(
    turret->SetAngleCommand(-PositioningConstants::kHighSideTurretAngle),
    elevator->SetHeightCommand(PositioningConstants::kHighElevatorHeight),
    pivot->SetAngleCommand(PositioningConstants::kHighSidePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kHighSideExtensionLength)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::ChargePlatformPositionCommand(Elevator* elevator, Extension* extension, Pivot* pivot) {
  return frc2::cmd::Sequence(
    pivot->SetAngleCommand(PositioningConstants::kChargePivotAngle),
    extension->SetExtensionCommand(PositioningConstants::kChargeExtensionLength),
    elevator->SetHeightCommand(PositioningConstants::kChargeElevatorHeight)
  ).WithName("PositioningCommandInProgress");
}

frc2::CommandPtr Positioning::DropCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition),
    frc2::cmd::Wait(PositioningConstants::kStowDelay),
    StowCommand(elevator, extension, pivot, claw)
  ).WithName("PositioningCommandInProgress");
}