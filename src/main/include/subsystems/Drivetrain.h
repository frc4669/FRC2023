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

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain(frc::Field2d* field);

  void ResetEncoders();

  void ConfigureMotor(WPI_TalonFX &motor, bool isInverted);

  void CurvatureDrive(double forward, double rotation); 

  frc2::CommandPtr DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation, std::function<double()> elevatorHeight);

  frc2::CommandPtr BoostCommand(double boost);

  frc::Pose2d OdometryPose();

  void ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation);

  units::degree_t GetYaw();

  units::meter_t GetLeftDistance();
  units::meter_t GetRightDistance();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void TankDriveVolts(units::volt_t left, units::volt_t right);

  void Periodic() override;

 private:
  WPI_TalonFX m_leftMain { DriveConstants::kLeftMain };
  WPI_TalonFX m_leftSecondary { DriveConstants::kLeftSecondary };
  WPI_TalonFX m_rightMain { DriveConstants::kRightMain };
  WPI_TalonFX m_rightSecondary { DriveConstants::kRightSecondary };

  frc::MotorControllerGroup m_leftMotors { m_leftMain, m_leftSecondary };
  frc::MotorControllerGroup m_rightMotors { m_rightMain, m_rightSecondary };

  frc::DifferentialDrive m_drive { m_leftMotors, m_rightMotors };

  double m_boost = 0.3;

  AHRS m_IMU { frc::SPI::Port::kMXP };
  units::degree_t m_yawOffset = 0_deg;

  frc::DifferentialDriveOdometry m_odometry { frc::Rotation2d(), 0_m, 0_m, frc::Pose2d() };
  frc::Field2d* m_field;
};
