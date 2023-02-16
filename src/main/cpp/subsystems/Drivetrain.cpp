// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(frc::Field2d* field) : m_field(field) {
    // Disable safety on the drivetrain motors
  m_drive.SetSafetyEnabled(false);

  // Configure the drivetrain motors (for now)
  ConfigureMotor(m_leftMain, false);
  ConfigureMotor(m_leftSecondary, false);
  m_leftSecondary.Follow(m_leftMain); // set back left motor to follow the front left motor

  ConfigureMotor(m_rightMain, true);
  ConfigureMotor(m_rightSecondary, true);
  m_rightSecondary.Follow(m_rightMain); // set back right motor to follow the front right motor
};

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
  m_odometry.Update(frc::Rotation2d(GetYaw()), GetLeftDistance(), GetRightDistance());

  frc::Pose2d robotPose = m_odometry.GetPose();

  m_field->SetRobotPose(robotPose);

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().value());
  frc::SmartDashboard::PutBoolean("Boost", m_boost == 1.0);

  frc::DifferentialDriveWheelSpeeds wheelSpeeds = GetWheelSpeeds();
  frc::SmartDashboard::PutNumber("Left Velocity", wheelSpeeds.left());
  frc::SmartDashboard::PutNumber("Right Velocity", wheelSpeeds.right());

  frc::SmartDashboard::PutNumber("Left Tick Vel", m_leftMain.GetSensorCollection().GetIntegratedSensorVelocity());
  frc::SmartDashboard::PutNumber("Right Tick Vel", m_rightMain.GetSensorCollection().GetIntegratedSensorVelocity());
}

void Drivetrain::ConfigureMotor(WPI_TalonFX &motor, bool isInverted) {
  // set the max velocity and acceleration for motion magic
  motor.ConfigMotionCruiseVelocity(20000);
  motor.ConfigMotionAcceleration(7000);

  // set the current limit for the supply/output current
  motor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
  motor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 25, 25, 0.5));

  // time it takes for the motor to go from 0 to full power (in seconds) in an open/closed loop
  motor.ConfigOpenloopRamp(0.1);
  motor.ConfigClosedloopRamp(0);

  // when controller is neutral, set motor to break
  motor.SetNeutralMode(NeutralMode::Brake);

  // disable motor safety
  motor.SetSafetyEnabled(false);

  // motor set experation time
  motor.SetExpiration(100_ms);

  motor.SetInverted(isInverted);

  // Motor PID values (for now)
  // motor.Config_kP(0, 0.00); 
  // motor.Config_kD(0, 0.00); 
  // motor.Config_kF(0, 0.00); 
}

void Drivetrain::CurvatureDrive(double speed, double rotation) {
  m_drive.CurvatureDrive(speed, rotation, !OperatorConstants::kCanTurnInPlace); 
}

units::degree_t Drivetrain::GetYaw() {
  return units::degree_t(-m_IMU.GetYaw()) - m_yawOffset;
}

frc2::CommandPtr Drivetrain::BoostCommand(double boost) {
  return this->RunOnce(
    [this, boost] { this->m_boost = boost; }
  );
}

frc2::CommandPtr Drivetrain::DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation) {
  return Run([this, speed = std::move(speed), rotation = std::move(rotation)] {
    CurvatureDrive(speed() * this->m_boost, rotation());
  }).WithName("DefaultDriveCommand");
}

units::meter_t Drivetrain::GetLeftDistance() {
  return units::inch_t(m_leftMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick);
}

units::meter_t Drivetrain::GetRightDistance() {
  return -units::inch_t(m_rightMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick);
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  units::meters_per_second_t leftVelocity = units::meters_per_second_t(units::meter_t(units::inch_t(
    m_leftMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * DriveConstants::kInchesPerTick
  )).value());

  units::meters_per_second_t rightVelocity = -units::meters_per_second_t(units::meter_t(units::inch_t(
    m_rightMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * DriveConstants::kInchesPerTick
  )).value());

  return { leftVelocity, rightVelocity };
}

void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMain.SetVoltage(left);
  m_rightMain.SetVoltage(right);
  m_leftSecondary.SetVoltage(left);
  m_rightSecondary.SetVoltage(right);
}

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation) {
  ResetEncoders();
  m_IMU.ZeroYaw();
  m_yawOffset = rotation.Degrees();
  m_odometryReset++;
  m_odometry.ResetPosition(rotation, 0_m, 0_m, pose);
}

void Drivetrain::ResetEncoders() {
  m_leftMain.GetSensorCollection().SetIntegratedSensorPosition(0);
  m_rightMain.GetSensorCollection().SetIntegratedSensorPosition(0);
}

frc::Pose2d Drivetrain::OdometryPose() {
  return m_odometry.GetPose();
}