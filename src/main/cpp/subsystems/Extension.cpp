// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Extension.h"

Extension::Extension() {
  m_mainMotor.ConfigMotionCruiseVelocity(30000);
  m_mainMotor.ConfigMotionAcceleration(8000);

  m_mainMotor.Config_kP(0, ExtensionConstants::kp);
  m_mainMotor.Config_kI(0, ExtensionConstants::ki);
  m_mainMotor.Config_kD(0, ExtensionConstants::kd);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(false);

  m_mainMotor.OverrideLimitSwitchesEnable(true);
};

void Extension::Periodic() {}

units::inch_t Extension::GetExtension() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * ExtensionConstants::kInchesPerTick);
}

units::meters_per_second_t Extension::GetVelocity() {
  return units::inch_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * ExtensionConstants::kInchesPerTick) / 1_s;
}

void Extension::SetExtension(units::inch_t extension) {
  double ticks = extension.value() / ExtensionConstants::kInchesPerTick;
  m_mainMotor.Set(TalonFXControlMode::MotionMagic, ticks); 
}

frc2::CommandPtr Extension::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
      m_isHomed = true; 
    });
}

frc2::CommandPtr Extension::SetExtensionCommand(units::inch_t extension) {
  return frc2::cmd::Either(
    Run([] {})
      .BeforeStarting([this, extension] { SetExtension(extension); })
      .Until([this, extension] {
        return units::math::abs(GetExtension() - extension) < ExtensionConstants::kPositionThreshold
          && units::math::abs(GetVelocity()) < ExtensionConstants::kVelocityThreshold;
      }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}