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
    constexpr int kTurretS = 2;
    constexpr int kTurretW = 1;
    constexpr int kTurretN = 3;
    constexpr int kTurretE = 4;

    constexpr int kPickupCone = 5;
    constexpr int kPickupShelf = 6;
    constexpr int kStow = 7;
    constexpr int kPickupCube = 8;
    constexpr int kPickupGround = 9;

    constexpr int kScoreLowCenter = 4;
    constexpr int kScoreMidCenter = 3;
    constexpr int kScoreMidLeft = 2;
    constexpr int kScoreMidRight = 6;
    constexpr int kScoreHighLeft = 1;
    constexpr int kScoreHighRight = 5;

    // constexpr int kDrop = 7;
  }

  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 0.7;
}

namespace CAN {
  // Drivetrain motors
  constexpr int kDrivetrainLeftMain = 21;
  constexpr int kDrivetrainLeftSecondary = 22;
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
  constexpr auto ks = 0.096137_V;
  constexpr auto kv = 2.3718_V * 1_s / 1_m;
  constexpr auto ka = 0.39708_V * 1_s * 1_s / 1_m;

  // PID controller gains
  constexpr double kp = 2.4087;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Physical parameters
  constexpr auto kTrackWidth = 20.75_in; // VERIFY
  constexpr double kGearRatio = 11.25;
  constexpr double kWheelCircumference = 0.1524 * 3.141592; // 6 in
  constexpr double kMetersPerTick = kWheelCircumference / (2048 * kGearRatio);

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
  constexpr auto kLimitSwitchSeparation = 30_in;

  // Elevator PID controller gains
  constexpr double kp = 1.3;
  constexpr double ki = 0;
  constexpr double kd = 0.08;

  // Elevator feedforward gains
  constexpr auto ks = 0.021916_V;
  constexpr auto kv = 1.3565_V * 1_s / 1_in;
  constexpr auto ka = 0.074867_V * 1_s * 1_s / 1_in;
  constexpr auto kg = -0.088634_V;

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
  constexpr double kp = 0.24;
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
  constexpr double kp = 0.014;
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
  constexpr double kp = 0.0004;
  constexpr double ki = 0;
  constexpr double kd = 0.0000125;

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
  constexpr auto kSafePivotHeight = 18.5_in;
  constexpr auto kClearPivotAngle = 145_deg;
  constexpr auto kStowDelay = 1_s;

  constexpr auto kShelfElevatorHeight = 29_in;
  constexpr auto kShelfExtensionLength = 0_in;
  constexpr auto kShelfPivotAngle = 130_deg;

  constexpr auto kGroundElevatorHeight = 6_in;
  constexpr auto kGroundExtensionLength = 12_in;
  constexpr auto kGroundPivotAngle = 70_deg;

  constexpr auto kStowElevatorHeight = 19_in;
  constexpr auto kStowExtensionLength = 0_in;
  constexpr auto kStowPivotAngle = 40_deg;

  constexpr auto kLowElevatorHeight = 21_in;
  constexpr auto kLowExtensionLength = 6_in;
  constexpr auto kLowPivotAngle = 70_deg;
  constexpr auto kLowTurretAngle = 0_deg;

  constexpr auto kMidElevatorHeight = 28_in;

  constexpr auto kMidCenterExtensionLength = 12_in;
  constexpr auto kMidCenterPivotAngle = 155_deg;
  constexpr auto kMidCenterTurretAngle = 0_deg;

  constexpr auto kMidSideExtensionLength = 0_in;
  constexpr auto kMidSidePivotAngle = 130_deg;
  constexpr auto kMidSideTurretAngle = 21_deg;

  constexpr auto kHighElevatorHeight = 29_in;
  constexpr auto kHighSideExtensionLength = 19_in;
  constexpr auto kHighSidePivotAngle = 155_deg;
  constexpr auto kHighSideTurretAngle = 13_deg;
}