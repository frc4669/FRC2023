// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/Phoenix.h>
#include <AHRS.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc2/command/CommandPtr.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void ResetEncoders();

  void ConfigureMotor(WPI_TalonFX &motor, bool isInverted);

  void CurvatureDrive(double forward, double rotation); 

  frc2::CommandPtr DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation);

  void ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_leftMain{ DriveConstants::kLeftMain };
  WPI_TalonFX m_leftSecondary{ DriveConstants::kLeftSecondary };
  WPI_TalonFX m_rightMain{ DriveConstants::kRightMain };
  WPI_TalonFX m_rightSecondary{ DriveConstants::kRightSecondary };

  frc::MotorControllerGroup m_leftMotors{ m_leftMain, m_leftSecondary };
  frc::MotorControllerGroup m_rightMotors{ m_rightMain, m_rightSecondary };

  frc::DifferentialDrive m_drive{ m_leftMotors, m_rightMotors };

  AHRS m_IMU{ frc::SPI::Port::kMXP };
};
