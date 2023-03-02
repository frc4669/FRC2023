// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>

namespace OperatorConstants {
  constexpr double kTurningSpeedMutiplier = 0.7; // slows down turning movement as joystick is too sensentive. 
} 

namespace DriveConstants {
  constexpr int kLeftMain = 21;          // Leading left motor
  constexpr int kLeftSecondary = 22;     // Following left motor

  constexpr int kRightMain = 11;         // Leading right motor
  constexpr int kRightSecondary = 12;    // Following right motor

  constexpr auto ks = 0.10013_V; // 0.10013
  constexpr auto kv = 2.5964_V * 1_s / 1_m; // 2.5964
  constexpr auto ka = 0.28575_V * 1_s * 1_s / 1_m; // 0.28575

  constexpr double kp = 2.5; // 2.1589 sysid
  constexpr double ki = 0;
  constexpr double kd = 0;

  constexpr auto kTrackWidth = 20.75_in;

  constexpr double kGearRatio = 11.25;
  constexpr double kWheelCircumference = 6 * 3.141592;
  constexpr double kInchesPerTick = kWheelCircumference / (2048 * kGearRatio);

  constexpr auto kMaxAutoSpeed = 3_mps;
  constexpr auto kMaxAutoAccel = 2_mps_sq;
}

namespace VerticalElevatorConstants {
  constexpr int kElevatorID = 51;

  constexpr double kElevatorGearRatio = 50;
  constexpr double kElevatorTeeth = 15;
  constexpr double kElevatorInchesPerTick = (kElevatorTeeth * 0.25) / (kElevatorGearRatio * 2048);

  constexpr double kElevatorP = 0.001;
  constexpr double kElevatorI = 0;
  constexpr double kElevatorD = 0;

  constexpr auto kShelfHeight = 10_in;
  constexpr auto kGroundHeight = 1_in;

  constexpr auto kElevatorSetpointThreshold = 3_in;
}

namespace HorizontalElevatorConstants {
  constexpr int kElevatorID = 52;

  constexpr double kElevatorGearRatio = 50; // place holder 
  constexpr double kElevatorTeeth = 15; // place holder
  constexpr double kElevatorInchesPerTick = (kElevatorTeeth * 0.25) / (kElevatorGearRatio * 2048);

  constexpr double kElevatorP = 1;
  constexpr double kElevatorI = 0;
  constexpr double kElevatorD = 0;

  constexpr auto kElevatorSetpointThreshold = 3_in;
}

namespace PivotConstants {
  constexpr int kPivotID = 53; 

  constexpr double kPivotGearRatio = 30; 
  constexpr double kPivotDegreesPerTick = 360.0 / 4096.0; 

  constexpr double kPivotP = 1;
  constexpr double kPivotI = 0;
  constexpr double kPivotD = 0;

  // 0 when limit active 
  constexpr double kMaxAngle = 180; 
}

namespace TurretConstants {
  constexpr int kTurretID = 50;

  constexpr double kTurretGearRatio = 3.2727275; 
  constexpr double kTurretDegreesPerTick = 360 / (4096 * kTurretGearRatio);

  constexpr double kTurretP = 1.3e-3;
  constexpr double kTurretI = 0;
  constexpr double kTurretD = 3.6052e-5;

  constexpr auto kTurretS = 0.58412_V;
  constexpr auto kTurretV = 0.0063192_V * 1_s / 1_m;
  constexpr auto kTurretA = 0.0011897_V * 1_s * 1_s / 1_m;

  constexpr double kTurretFwdThreshold = 180 / kTurretDegreesPerTick;
  constexpr double kTurretRevThreshold = -180 / kTurretDegreesPerTick;
}

namespace PickupControlBoardConst {
  // turret control
  constexpr int turretNorth = 1;
  constexpr int turretSouth = 2;
  constexpr int turretEast = 3; 
  constexpr int turretWest = 4;

  // pickup
  constexpr int pickupGroundLevel = 5;
  constexpr int pickupShelfLevel = 6;
  constexpr int pickupStowBetween = 7;
  constexpr int pickupSelectCone = 8;
  constexpr int pickupSelectCube = 9;
}

namespace ScoringControlBoardConst {
  // scoring selection
  constexpr int scoringExtLow = 1;
  constexpr int scoringExtMid = 2;
  constexpr int scoringIntLeftMid = 3;
  constexpr int scoringIntLeftHigh = 4;
  constexpr int scoringIntRightMid = 5;
  constexpr int scoringIntRightHigh = 6;
}
