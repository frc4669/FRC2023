// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Claw.h"

Claw::Claw() {
  m_controlSolenoid.Set(frc::DoubleSolenoid::kOff);
  m_pressureSolenoid.Set(frc::DoubleSolenoid::kReverse);
}

frc2::CommandPtr Claw::TogglePressureCommand(bool pressure) {
  return RunOnce([this, pressure] {
    if (pressure == ClawConstants::kConePressure) m_pressureSolenoid.Set(frc::DoubleSolenoid::kForward);
    else m_pressureSolenoid.Set(frc::DoubleSolenoid::kReverse);
  });
}

frc2::CommandPtr Claw::TogglePressureCommand() {
  return RunOnce([this] {
    m_pressureSolenoid.Toggle();
  });
}

frc2::CommandPtr Claw::ToggleActivationStateCommand(bool state) {
  return RunOnce([this, state] {
    if (state == ClawConstants::kClosePosition) m_controlSolenoid.Set(frc::DoubleSolenoid::kForward);
    else m_controlSolenoid.Set(frc::DoubleSolenoid::kReverse);
  });
}

frc2::CommandPtr Claw::ToggleActivationStateCommand() {
  return RunOnce([this] {
    m_controlSolenoid.Toggle();
  });
}

bool Claw::GetPressure() {
  return m_pressureSolenoid.Get() == frc::DoubleSolenoid::kForward ? ClawConstants::kConePressure : ClawConstants::kCubePressure;
}

bool Claw::GetActivationState() {
  return m_controlSolenoid.Get() == frc::DoubleSolenoid::kForward ? ClawConstants::kClosePosition : ClawConstants::kOpenPosition;
}

void Claw::DisableCompressor() {
  m_compressor.Disable(); 
}

void Claw::Periodic() {
  frc::SmartDashboard::PutBoolean("Cone Pressure", GetPressure() == ClawConstants::kConePressure);
  frc::SmartDashboard::PutBoolean("Cube Pressure", GetPressure() == ClawConstants::kCubePressure);
  frc::SmartDashboard::PutBoolean("Closed", GetActivationState() == ClawConstants::kClosePosition);
}