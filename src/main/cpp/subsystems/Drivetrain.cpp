// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {
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
void Drivetrain::Periodic() {}

void Drivetrain::ConfigureMotor(WPI_TalonFX &motor, bool isInverted) {
  // set the max velocity and acceleration for motion magic
  motor.ConfigMotionCruiseVelocity(20000);
  motor.ConfigMotionAcceleration(7000);

  // set the current limit for the supply/output current
  motor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
  motor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 25, 25, 0.5));

  // time it takes for the motor to go from 0 to full power (in seconds) in an open/closed loop
  motor.ConfigOpenloopRamp(0.2);
  motor.ConfigClosedloopRamp(0);

  // when controller is neutral, set motor to break
  motor.SetNeutralMode(NeutralMode::Brake);

  // disable motor safety
  motor.SetSafetyEnabled(false);

  // motor set experation time
  motor.SetExpiration(100_ms);

  motor.SetInverted(isInverted);

  // Motor PID values (for now)
  motor.Config_kP(0, 0.01); 
  motor.Config_kD(0, 0.00); 
  motor.Config_kF(0, 0.00); 
}

void Drivetrain::CurvatureDrive(double speed, double rotation) {
  m_drive.CurvatureDrive(speed, rotation, OperatorConstants::kCanTurnInPlace); 
}

frc2::CommandPtr Drivetrain::DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation) {
  return Run([this, speed = std::move(speed), rotation = std::move(rotation)] {
    CurvatureDrive(speed(), rotation());
  }).WithName("DefaultDriveCommand");
}