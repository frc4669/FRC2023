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
  // Controller IDs
  constexpr int kDriverController = 0;
  constexpr int kOperatorController = 1;
  constexpr int kButtonBoardA = 2;
  constexpr int kButtonBoardB = 3;

  // Button board button IDs
  // Turret and pickup control on board A, score control on board B
  namespace ButtonBoard {
    constexpr int kTurretS = 1;
    constexpr int kTurretW = 2;
    constexpr int kTurretN = 3;
    constexpr int kTurretE = 4;

    constexpr int kPickupShelf = 5;
    constexpr int kPickupGround = 6;
    constexpr int kPickupCone = 7;
    constexpr int kPickupCube = 8;
    constexpr int kStow = 9;

    constexpr int kScoreLowCenter = 1;
    constexpr int kScoreMidCenter = 2;
    constexpr int kScoreMidLeft = 3;
    constexpr int kScoreMidRight = 4;
    constexpr int kScoreHighLeft = 5;
    constexpr int kScoreHighRight = 6;
  }

  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 0.7;
} 

namespace DriveConstants {
  // Drive motor CAN IDs
  constexpr int kLeftMain = 21;
  constexpr int kLeftSecondary = 22;
  constexpr int kRightMain = 11;
  constexpr int kRightSecondary = 12;

  // Feedforward gains
  constexpr auto ks = 0.10013_V;
  constexpr auto kv = 2.5964_V * 1_s / 1_m;
  constexpr auto ka = 0.28575_V * 1_s * 1_s / 1_m;

  // PID controller gains
  constexpr double kp = 2.1589;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Physical parameters
  constexpr auto kTrackWidth = 20.75_in;
  constexpr double kGearRatio = 11.25;
  constexpr double kWheelCircumference = 6 * 3.141592;
  constexpr double kInchesPerTick = kWheelCircumference / (2048 * kGearRatio);

  // Autonomous parameters
  constexpr auto kMaxAutoSpeed = 3_mps;
  constexpr auto kMaxAutoAccel = 2_mps_sq;
}

namespace VerticalElevatorConstants {
  // Elevator motor CAN ID
  constexpr int kElevatorID = 51;

  // Elevator physical parameters
  constexpr double kElevatorGearRatio = 50;
  constexpr double kElevatorTeeth = 15;
  constexpr double kElevatorInchesPerTick = (kElevatorTeeth * 0.25) / (kElevatorGearRatio * 2048);

  // Elevator PID controller gains
  constexpr double kElevatorP = 0.001;
  constexpr double kElevatorI = 0;
  constexpr double kElevatorD = 0;

  // ???
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
  constexpr double kPivotDegreesPerTick = 360 / (4096); 

  constexpr double kPivotP = 0.001;
  constexpr double kPivotI = 0;
  constexpr double kPivotD = 0;

  // 0 when limit active 
  constexpr double kMaxAngle = 180; 
}

namespace TurretConstants {
  // Turret motor CAN ID
  constexpr int kTurretID = 50;

  // Turret physical parameters
  constexpr double kTurretGearRatio = 3.2727275; 
  constexpr double kTurretDegreesPerTick = 360 / (4096 * kTurretGearRatio);

  // Turret PID controller gains
  constexpr double kTurretP = 0.0008;
  constexpr double kTurretI = 0;
  constexpr double kTurretD = 0.000025;

  // Turret feedforward gains
  constexpr auto kTurretS = 0_V; // 0.58412_V;
  constexpr auto kTurretV = 0.0063192_V * 1_s / 1_deg;
  constexpr auto kTurretA = 0_V * 1_s * 1_s / 1_deg; // 0.0011897_V * 1_s * 1_s / 1_m;

  // Turret rotation limits
  constexpr double kTurretFwdThreshold = 180 / kTurretDegreesPerTick;
  constexpr double kTurretRevThreshold = -180 / kTurretDegreesPerTick;
}