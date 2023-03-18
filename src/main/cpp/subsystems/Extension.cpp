// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Extension.h"

Extension::Extension() {
  m_mainMotor.ConfigMotionCruiseVelocity(30000);
  m_mainMotor.ConfigMotionAcceleration(8000);

  // m_mainMotor.Config_kP(0, ExtensionConstants::kp);
  // m_mainMotor.Config_kI(0, ExtensionConstants::ki);
  // m_mainMotor.Config_kD(0, ExtensionConstants::kd);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(false);

  m_mainMotor.OverrideLimitSwitchesEnable(true); // Reverse limit switch = full extension
  
  frc::SmartDashboard::PutData(&m_controller);
};

void Extension::Periodic() {
  frc::SmartDashboard::PutNumber("Extension (in)", GetExtension().value());
}

units::inch_t Extension::GetExtension() {
  return -units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ExtensionConstants::kInchesPerTick);
}

units::meters_per_second_t Extension::GetVelocity() {
  return -units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * ExtensionConstants::kInchesPerTick) / 1_s;
}

frc2::CommandPtr Extension::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0.2); })
    .Until([this] { return m_mainMotor.IsFwdLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
      m_isHomed = true; 
    });
}

frc2::CommandPtr Extension::SetExtensionCommand(units::inch_t extension) {
  return frc2::cmd::Either(
    Run([this, extension] {
      double currentExtension = GetExtension().value();
      double output = -m_controller.Calculate(currentExtension);
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, output);
    })
    .BeforeStarting([this, extension] {
      m_controller.Reset();
      m_controller.SetSetpoint(extension.value());
    })
    .Until([this, extension] {
      return units::math::abs(GetExtension() - extension) < ExtensionConstants::kPositionThreshold
      && units::math::abs(GetVelocity()) < ExtensionConstants::kVelocityThreshold;
    }).AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}

frc2::CommandPtr Extension::SetHomedCommand() {
  return RunOnce([this] { m_isHomed = true; });
}