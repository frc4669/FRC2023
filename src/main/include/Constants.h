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

    constexpr int kDrop = 7;
  }

  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 0.7;
}

namespace CAN {
  // Drivetrain motors
  constexpr int kDrivetrainLeftMain = 21;
  constexpr int kDrivetrainLeftSecondary = 11;
  constexpr int kDrivetrainRightMain = 11;
  constexpr int kDrivetrainRightSecondary = 12;

  // Turret motor
  constexpr int kTurretMain = 50;

  // Elevator motor
  constexpr int kElevatorMain = 51;

  // Extension motor
  constexpr int kExtensionMain = 52;

  // Pivot motor
  constexpr int kPivotMain = 53;
}

namespace DriveConstants {
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

  // Autonomous balancing parameters
  constexpr double kBalanceInitialSpeed = 0.3;
  constexpr auto kBalanceThresholdA = 20_deg;
  constexpr double kBalanceSlowSpeed = 0.15;
  constexpr auto kBalanceThresholdB = 10_deg;
  constexpr auto kBalanceMaxControllerSpeed = 0.1;
  constexpr double kBalanceP = 0.01;
  constexpr auto kLevelThreshold = 1_deg;
}

namespace ClawConstants {
  // Pressure selection port IDs
  constexpr int kPressureOpenID = 6;
  constexpr int kPressureCloseID = 7;

  // Activation state port IDs
  constexpr int kControlOpenID = 0;
  constexpr int kControlCloseID = 1;

  // "Enumerations" of claw states
  constexpr bool kConePressure = true;
  constexpr bool kCubePressure = false;
  constexpr bool kOpenPosition = true;
  constexpr bool kClosePosition = false;
}

namespace ElevatorConstants {
  // Elevator physical parameters
  constexpr double kGearRatio = 50;
  constexpr double kTeeth = 16;
  constexpr double kInchesPerTick = (kTeeth * 0.25) / (kGearRatio * 2048);

  // Elevator PID controller gains
  constexpr double kp = 0.001;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Elevator setpoint thresholds
  constexpr auto kPositionThreshold = 4_in;
  constexpr auto kVelocityThreshold = 1.5_in / 1_s;
}

namespace ExtensionConstants {
  // Extension physical parameters
  constexpr double kGearRatio = 30;
  constexpr double kTeeth = 16;
  constexpr double kInchesPerTick = (kTeeth * 0.25) / (kGearRatio * 2048);

  // Extension PID controller gains
  constexpr double kp = 1;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Extension setpoint thresholds
  constexpr auto kPositionThreshold = 3_in;
  constexpr auto kVelocityThreshold = 1_in / 1_s;
}

namespace PivotConstants {
  // Wrist pivot physical parameters
  constexpr double kGearRatio = 200;
  constexpr double kDegreesPerTick = 360 / (kGearRatio * 2048); 

  // Wrist pivot PID controller gains
  constexpr double kp = 1;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Wrist pivot setpoint thresholds
  constexpr auto kPositionThreshold = 8_deg;
  constexpr auto kVelocityThreshold = 4_deg / 1_s;
}

namespace TurretConstants {
  // Turret physical parameters
  constexpr double kGearRatio = 3.2727275; 
  constexpr double kDegreesPerTick = 360 / (4096 * kGearRatio);

  // Turret PID controller gains
  constexpr double kp = 0.0008;
  constexpr double ki = 0;
  constexpr double kd = 0.000025;

  // Turret feedforward gains
  constexpr auto ks = 0_V; // 0.58412_V;
  constexpr auto kv = 0.0063192_V * 1_s / 1_deg;
  constexpr auto ka = 0_V * 1_s * 1_s / 1_deg; // 0.0011897_V * 1_s * 1_s / 1_m;

  // Turret rotation limits
  constexpr double kFwdThreshold = 180 / kDegreesPerTick;
  constexpr double kRevThreshold = -180 / kDegreesPerTick;

  // Turret setpoint thresholds
  constexpr auto kVelocityThreshold = 8_deg / 1_s;
  constexpr auto kPositionThreshold = 4_deg;
}

namespace PositioningConstants {
  constexpr auto kSafePivotHeight = 0_in; // TODO
  constexpr auto kDropDelta = 0_in; // TODO
  constexpr auto kStowDelay = 0_s; // TODO
  constexpr auto kDropDelay = 0_s; // TODO

  constexpr auto kShelfElevatorHeight = 0_in; // TODO
  constexpr auto kShelfExtensionLength = 0_in; // TODO
  constexpr auto kShelfPivotAngle = 0_deg; // TODO

  constexpr auto kGroundElevatorHeight = 0_in; // TODO
  constexpr auto kGroundExtensionLength = 0_in; // TODO
  constexpr auto kGroundPivotAngle = 0_deg; // TODO

  constexpr auto kStowElevatorHeight = 0_in; // TODO
  constexpr auto kStowExtensionLength = 0_in; // TODO
  constexpr auto kStowPivotAngle = 0_deg; // TODO

  constexpr auto kLowElevatorHeight = 0_in; // TODO
  constexpr auto kLowExtensionLength = 0_in; // TODO
  constexpr auto kLowPivotAngle = 0_deg; // TODO
  constexpr auto kLowTurretAngle = 0_deg;

  constexpr auto kMidElevatorHeight = 0_in; // TODO

  constexpr auto kMidCenterExtensionLength = 0_in; // TODO
  constexpr auto kMidCenterPivotAngle = 0_deg; // TODO
  constexpr auto kMidCenterTurretAngle = 0_deg;

  constexpr auto kMidSideExtensionLength = 0_in; // TODO
  constexpr auto kMidSidePivotAngle = 0_deg; // TODO
  constexpr auto kMidSideTurretAngle = 21_deg;

  constexpr auto kHighElevatorHeight = 0_in; // TODO
  constexpr auto kHighSideExtensionLength = 0_in; // TODO
  constexpr auto kHighSidePivotAngle = 0_deg; // TODO
  constexpr auto kHighSideTurretAngle = 13_deg;
}