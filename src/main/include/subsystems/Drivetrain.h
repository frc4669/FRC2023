// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain(frc::Field2d* field);
  void Periodic() override;

  void ConfigureMotor(WPI_TalonFX &motor, bool isInverted);

  void ResetEncoders();
  void ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation);

  void CurvatureDrive(double forward, double rotation);
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  frc2::CommandPtr DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation, std::function<double()> elevatorHeight);
  frc2::CommandPtr BoostCommand(double boost);
  frc2::CommandPtr AutomaticBalanceCommand();

  double AutomaticBalance();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  units::meter_t GetLeftDistance();
  units::meter_t GetRightDistance();
  units::degree_t GetYaw();
  units::degree_t GetPitch();
  frc::Pose2d OdometryPose();

 private:
  WPI_TalonFX m_leftMain { CAN::kDrivetrainLeftMain };
  WPI_TalonFX m_leftSecondary { CAN::kDrivetrainLeftSecondary };
  WPI_TalonFX m_rightMain { CAN::kDrivetrainRightMain };
  WPI_TalonFX m_rightSecondary { CAN::kDrivetrainRightSecondary };

  frc::MotorControllerGroup m_leftMotors { m_leftMain, m_leftSecondary };
  frc::MotorControllerGroup m_rightMotors { m_rightMain, m_rightSecondary };

  frc::DifferentialDrive m_drive { m_leftMotors, m_rightMotors };

  frc::DifferentialDriveOdometry m_odometry { frc::Rotation2d(), 0_m, 0_m, frc::Pose2d() };
  frc::Field2d* m_field;

  AHRS m_IMU { frc::SPI::Port::kMXP };
  units::degree_t m_yawOffset = 0_deg;

  double m_boost = 0.3;

  frc2::PIDController m_balanceController { DriveConstants::kBalanceP, 0, 0 };
};
