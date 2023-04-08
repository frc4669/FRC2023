// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
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

    constexpr int kDrop = 10;

    constexpr int kScoreLowCenter = 4;
    constexpr int kScoreMidCenter = 3;
    constexpr int kScoreMidLeft = 2;
    constexpr int kScoreMidRight = 6;
    constexpr int kScoreHighLeft = 1;
    constexpr int kScoreHighRight = 5;
  }

  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 1.0;
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
  constexpr auto kHomePosition = 19.68_in;

  // Elevator PID controller gains
  constexpr double kp = 21.8;
  constexpr double ki = 0;
  constexpr double kd = 0.1;

  // Elevator feedforward gains
  constexpr auto ks = 0.021916_V;
  constexpr auto kv = 1.3565_V * 1_s / 1_in;
  constexpr auto ka = 0.074867_V * 1_s * 1_s / 1_in;
  constexpr auto kg = -0.088634_V;

  // Elevator motion profile constraints
  constexpr auto kMaxVelocity = 1_mps;
  constexpr auto kMaxAccel = 1_mps_sq;

  // Elevator setpoint thresholds
  constexpr auto kPositionThreshold = 2_in;
  constexpr auto kVelocityThreshold = 1.5_in / 1_s;
}

namespace ExtensionConstants {
  // Extension physical parameters
  constexpr double kGearRatio = 30;
  constexpr double kTeeth = 16;
  constexpr double kInchesPerTick = (kTeeth * 0.25) / (kGearRatio * 2048);

  // Extension PID controller gains
  constexpr double kp = 13.5;
  constexpr double ki = 0;
  constexpr double kd = 0.15;

  // Extension motion profile constraints
  constexpr auto kMaxVelocity = 1_mps;
  constexpr auto kMaxAccel = 1_mps_sq;

  // Extension setpoint thresholds
  constexpr auto kPositionThreshold = 3_in;
  constexpr auto kVelocityThreshold = 1_in / 1_s;
}

namespace PivotConstants {
  // Wrist pivot physical parameters
  constexpr double kGearRatio = 200;
  constexpr double kDegreesPerTick = 360 / (kGearRatio * 2048); 

  // Wrist pivot PID controller gains
  constexpr double kp = 0.24;
  constexpr double ki = 0;
  constexpr double kd = 0;

  // Wrist pivot feedforward gains
  constexpr auto ks = -0.019768_V;
  constexpr auto kv = 0.5_V * 1_s / 1_deg; // 0.061282
  constexpr auto ka = 0.00073196_V * 1_s * 1_s / 1_deg;
  constexpr auto kg = 0.031072_V;
  constexpr auto kHorizontalOffset = -166.74_deg;

  // LIFE RUINING CONSTRAINTS
  constexpr auto kMaxVelocity = 300000000_deg_per_s;
  constexpr auto kMaxAccel = 30000000_deg_per_s_sq;

  // Wrist pivot setpoint thresholds
  constexpr auto kPositionThreshold = 4_deg;
  constexpr auto kVelocityThreshold = 3_deg / 1_s;
}

namespace TurretConstants {
  // Turret physical parameters
  constexpr double kGearRatio = 72.0 / 16.0;
  constexpr double kDegreesPerTick = 360 / (4096 * kGearRatio);

  // Turret PID controller gains
  constexpr double kp = 0.02; // 1.0576
  constexpr double ki = 0;
  constexpr double kd = 0; // 0.079649

  // Turret motion profile constraints
  constexpr auto kMaxVelocity = 180_deg_per_s;
  constexpr auto kMaxAccel = 160_deg_per_s_sq;

  // Turret feedforward gains
  constexpr auto ks = 0.68611_V;
  constexpr auto kv = 0.087389_V * 1_s / 1_deg;
  constexpr auto ka = 0.0029981_V * 1_s * 1_s / 1_deg;

  // Turret rotation limits
  constexpr auto kFwdThreshold = 90_deg;
  constexpr auto kRevThreshold = -180_deg;

  // Turret setpoint thresholds
  constexpr auto kVelocityThreshold = 2_deg / 1_s;
  constexpr auto kPositionThreshold = 3_deg;
}

namespace PositioningConstants {
  constexpr auto kDropDistance = 0.04_m;

  constexpr auto kSafePivotHeight = 18.5_in;
  constexpr auto kClearPivotAngle = 145_deg;
  constexpr auto kStowDelay = 1_s;

  constexpr auto kShelfElevatorHeight = 28.7_in;
  constexpr auto kShelfExtensionLength = 0_in;
  constexpr auto kShelfPivotAngle = 125_deg; //130,110

  constexpr auto kGroundElevatorHeight = 3_in;
  constexpr auto kGroundExtensionLength = 12_in;
  constexpr auto kGroundPivotAngle = 70_deg;

  constexpr auto kStowElevatorHeight = 24_in;
  constexpr auto kStowExtensionLength = 0_in;
  constexpr auto kStowPivotAngle = 40_deg;

  constexpr auto kLowElevatorHeight = 18_in;
  constexpr auto kLowExtensionLength = 9_in;
  constexpr auto kLowPivotAngle = 70_deg;
  constexpr auto kLowTurretAngle = 0_deg;

  constexpr auto kMidElevatorHeight = 28.5_in;

  constexpr auto kMidCenterExtensionLength = 15_in;
  constexpr auto kMidCenterPivotAngle = 155_deg;
  constexpr auto kMidCenterTurretAngle = 0_deg;

  constexpr auto kMidSideExtensionLength = 3_in;
  constexpr auto kMidSidePivotAngle = 140_deg;
  constexpr auto kMidSideTurretAngle = 21_deg;

  constexpr auto kHighElevatorHeight = 28.5_in;
  constexpr auto kHighSideExtensionLength = 19_in;
  constexpr auto kHighSidePivotAngle = 155_deg;
  constexpr auto kHighSideTurretAngle = 13_deg;

  constexpr auto kChargePivotAngle = 150_deg;
  constexpr auto kChargeElevatorHeight = 3_in;
  constexpr auto kChargeExtensionLength = 0_in;
}