#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/Elevator.h"
#include "subsystems/HorizontalExtension.h"
#include "subsystems/Pivot.h"
#include "subsystems/Turret.h"
#include "subsystems/Claw.h"

namespace positioning {
  // Pressure configuration commands
  frc2::CommandPtr CubePickupSelectCommand(Claw* claw);
  frc2::CommandPtr ConePickupSelectCommand(Claw* claw);

  // Pickup and stow position commands
  frc2::CommandPtr ShelfPickupCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw);
  frc2::CommandPtr GroundPickupCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw);
  frc2::CommandPtr StowCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot);

  // L1 scoring commands
  frc2::CommandPtr ScoreLowCenterCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);

  // L2 scoring commands
  frc2::CommandPtr ScoreMidLeftCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);
  frc2::CommandPtr ScoreMidCenterCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);
  frc2::CommandPtr ScoreMidRightCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);

  // L3 scoring commands
  frc2::CommandPtr ScoreHighLeftCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);
  frc2::CommandPtr ScoreHighRightCommand(Elevator* elevator, HorizontalExtension* extension, Pivot* pivot, Claw* claw, Turret* turret);
}