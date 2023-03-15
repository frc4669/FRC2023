// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(frc::Field2d* field) : m_field(field) {
  m_drive.SetSafetyEnabled(false);

  ConfigureMotor(m_leftMain, false);
  ConfigureMotor(m_leftSecondary, false);
  m_leftSecondary.Follow(m_leftMain);

  ConfigureMotor(m_rightMain, true);
  ConfigureMotor(m_rightSecondary, true);
  m_rightSecondary.Follow(m_rightMain);
}

void Drivetrain::Periodic() {
  m_odometry.Update(frc::Rotation2d(GetYaw()), GetLeftDistance(), GetRightDistance());

  frc::Pose2d robotPose = m_odometry.GetPose();

  m_field->SetRobotPose(robotPose);

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().value());
  frc::SmartDashboard::PutBoolean("Boost", m_boost == 1.0);
}

void Drivetrain::ConfigureMotor(WPI_TalonFX &motor, bool isInverted) {
  motor.ConfigFactoryDefault();

  // Motion Magic closed loop control configuration
  motor.ConfigMotionCruiseVelocity(20000);
  motor.ConfigMotionAcceleration(7000);

  // Motor current configuration (exceeding these limits generally damages the motor)
  motor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
  motor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 25, 25, 0.5));

  // Configures time taken to move motor from neutral to full power
  motor.ConfigOpenloopRamp(0.1);
  motor.ConfigClosedloopRamp(0);

  // Neutral mode configuration (sets motor to passively reduce motion without other input)
  motor.SetNeutralMode(NeutralMode::Brake);

  // Reverses default motor direction (needed if motor is orientated in an undesirable position)
  motor.SetInverted(isInverted);

  motor.SetSafetyEnabled(false);
}

void Drivetrain::ResetEncoders() {
  m_leftMain.GetSensorCollection().SetIntegratedSensorPosition(0);
  m_rightMain.GetSensorCollection().SetIntegratedSensorPosition(0);
}

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation) {
  ResetEncoders();
  m_IMU.ZeroYaw();
  m_yawOffset = rotation.Degrees();
  m_odometry.ResetPosition(rotation, 0_m, 0_m, pose);
}

void Drivetrain::CurvatureDrive(double speed, double rotation) {
  m_drive.CurvatureDrive(speed, rotation, false); 
}

void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMain.SetVoltage(left);
  m_rightMain.SetVoltage(right);
  m_leftSecondary.SetVoltage(left);
  m_rightSecondary.SetVoltage(right);
}

frc2::CommandPtr Drivetrain::DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation, std::function<double()> elevatorHeight) {
  return Run([this, speed = std::move(speed), rotation = std::move(rotation)] {
    CurvatureDrive(speed() * m_boost, rotation());
  });
}

frc2::CommandPtr Drivetrain::BoostCommand(double boost) {
  return RunOnce(
    [this, boost] { m_boost = boost; }
  );
}

frc2::CommandPtr Drivetrain::AutomaticBalanceCommand() {
  return Run([this] {
    m_drive.ArcadeDrive(AutomaticBalance(), 0, false);
  });
  
  // Likely do not need the below
  // .Until([this] {
  //   return units::math::abs(GetPitch()) <= DriveConstants::kLevelThreshold;
  // });
}

double Drivetrain::AutomaticBalance() {
  units::degree_t pitch = GetPitch();
  double direction = pitch <= 0_deg ? 1 : -1;
  
  if (units::math::abs(pitch) > DriveConstants::kBalanceThresholdA)
    return DriveConstants::kBalanceInitialSpeed * direction;
  else if (units::math::abs(pitch) > DriveConstants::kBalanceThresholdB)
    return DriveConstants::kBalanceSlowSpeed * direction;
  else return std::clamp(m_balanceController.Calculate(pitch.value(), 0), -DriveConstants::kBalanceMaxControllerSpeed, DriveConstants::kBalanceMaxControllerSpeed);
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

units::meter_t Drivetrain::GetLeftDistance() {
  return units::inch_t(m_leftMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick);
}

units::meter_t Drivetrain::GetRightDistance() {
  return -units::inch_t(m_rightMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick);
}

units::degree_t Drivetrain::GetYaw() {
  return units::degree_t(-m_IMU.GetYaw()) - m_yawOffset;
}

units::degree_t Drivetrain::GetPitch() {
  return units::degree_t(m_IMU.GetPitch());
}

frc::Pose2d Drivetrain::OdometryPose() {
  return m_odometry.GetPose();
}