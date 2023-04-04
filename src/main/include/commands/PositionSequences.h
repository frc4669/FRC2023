#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/Elevator.h"
#include "subsystems/Extension.h"
#include "subsystems/Pivot.h"
#include "subsystems/Turret.h"
#include "subsystems/Claw.h"

class Positioning {
 public:
  // Pressure configuration commands
  static frc2::CommandPtr CubePickupSelectCommand(Claw* claw);
  static frc2::CommandPtr ConePickupSelectCommand(Claw* claw);

  // Pickup and stow position commands
  static frc2::CommandPtr ShelfPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);
  static frc2::CommandPtr GroundPickupCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);
  static frc2::CommandPtr StowCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);

  // L1 scoring commands
  static frc2::CommandPtr ScoreLowCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);

  // L2 scoring commands
  static frc2::CommandPtr ScoreMidLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret);
  static frc2::CommandPtr ScoreMidCenterCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);
  static frc2::CommandPtr ScoreMidRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret);

  // L3 scoring commands
  static frc2::CommandPtr ScoreHighLeftCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret);
  static frc2::CommandPtr ScoreHighRightCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw, Turret* turret);

  // Charge station configuration command
  static frc2::CommandPtr ChargePlatformPositionCommand(Elevator* elevator, Extension* extension, Pivot* pivot);

  // Drop command
  static frc2::CommandPtr DropCommand(Elevator* elevator, Extension* extension, Pivot* pivot, Claw* claw);
};