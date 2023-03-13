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

frc2::CommandPtr positioning::ShelfPickupCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetDistanceCommandVertical(PositioningConstants::kShelfElevatorHeight),
    extension->SetDistanceCommandHorizontal(PositioningConstants::kShelfExtensionLength),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition)
  );
}

frc2::CommandPtr positioning::GroundPickupCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetDistanceCommandVertical(PositioningConstants::kSafePivotHeight),
    pivot->SetAngleCommand(PositioningConstants::kShelfPivotAngle),
    elevator->SetDistanceCommandVertical(PositioningConstants::kGroundElevatorHeight),
    extension->SetDistanceCommandHorizontal(PositioningConstants::kGroundExtensionLength),
    claw->ToggleActivationStateCommand(ClawConstants::kOpenPosition)
  );
}

frc2::CommandPtr positioning::StowCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw) {
  return frc2::cmd::Sequence(
    elevator->SetDistanceCommandVertical(PositioningConstants::kSafePivotHeight),
    pivot->SetAngleCommand(PositioningConstants::kStowPivotAngle),
    elevator->SetDistanceCommandVertical(PositioningConstants::kStowElevatorHeight),
    extension->SetDistanceCommandHorizontal(PositioningConstants::kStowExtensionLength)
  );
}