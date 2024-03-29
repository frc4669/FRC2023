// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Claw : public frc2::SubsystemBase {
 public:
  Claw();

  frc2::CommandPtr TogglePressureCommand(bool pressure);
  frc2::CommandPtr TogglePressureCommand();
  
  frc2::CommandPtr ToggleActivationStateCommand(bool state);
  frc2::CommandPtr ToggleActivationStateCommand();

  bool GetPressure();
  bool GetActivationState();

  void DisableCompressor(); 

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::Compressor m_compressor { frc::PneumaticsModuleType::REVPH };

  frc::DoubleSolenoid m_pressureSolenoid { frc::PneumaticsModuleType::REVPH, 6, 7 }; 
  frc::DoubleSolenoid m_controlSolenoid { frc::PneumaticsModuleType::REVPH, 1, 0 };
};
