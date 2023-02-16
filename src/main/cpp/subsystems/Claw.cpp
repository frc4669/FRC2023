// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Claw.h"

Claw::Claw() {
  m_openingControlSolenoid.Set(frc::DoubleSolenoid::kOff);
  m_pressureSolenoid.Set(frc::DoubleSolenoid::kOff);
  m_openingControlSolenoid.Set(frc::DoubleSolenoid::kForward);
  m_pressureSolenoid.Set(frc::DoubleSolenoid::kForward);
  // m_solenoid.Set(true);
  // m_solenoid.Set(false);
}

// frc2::CommandPtr Claw::ManipulateCommand(bool position) {
//   return RunOnce([this, position] {
//     frc::SmartDashboard::PutBoolean("IsOpened", position);
//     this->m_solenoid.Set(position);
//   });
// }

// true = high, false = low, nothing means toggle
frc2::CommandPtr Claw::SelectPressure() {
  return RunOnce([this] {
    this->m_pressureSolenoid.Toggle();

  });
}

frc2::CommandPtr Claw::ChangeActivationState() {
  return RunOnce([this] {
    this->m_openingControlSolenoid.Toggle();
  });
}

void Claw::DisableCompressor()
{
  m_phCompressor.Disable(); 
}

// This method will be called once per scheduler run
void Claw::Periodic() {
  
}