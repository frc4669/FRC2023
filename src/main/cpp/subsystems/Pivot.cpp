// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pivot.h"

Pivot::Pivot() { 
  m_mainMotor.ConfigMotionCruiseVelocity(7000);
  m_mainMotor.ConfigMotionAcceleration(3000);

  // m_mainMotor.Config_kP(0, PivotConstants::kp);
  // m_mainMotor.Config_kI(0, PivotConstants::ki);
  // m_mainMotor.Config_kD(0, PivotConstants::kd);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(false);

  m_mainMotor.OverrideLimitSwitchesEnable(true); // Forward = pivoted out
}

void Pivot::Periodic() {
  frc::SmartDashboard::PutNumber("Pivot Angle (deg)", GetAngle().value());
}

units::degree_t Pivot::GetAngle() {
  return units::degree_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * PivotConstants::kDegreesPerTick);
}

double Pivot::GetVelocity() {
  return m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * PivotConstants::kDegreesPerTick;
}

frc2::CommandPtr Pivot::HomeCommand() {
  return Run([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_mainMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
      m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
      m_isHomed = true;
    });
}

frc2::CommandPtr Pivot::SetAngleCommand(units::degree_t angle) {
  /*return frc2::cmd::Either(
    Run([] {})
      .BeforeStarting([this, angle] { SetAngle(angle); })
      .Until([this, angle] {
        return units::math::abs(GetAngle() - angle) < PivotConstants::kPositionThreshold
          && GetVelocity() < PivotConstants::kVelocityThreshold.value();
      }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );*/

  return frc2::cmd::Either(
    Run([this] {
      double currentAngle = GetAngle().value();
      double output = m_controller.Calculate(currentAngle);
      m_mainMotor.Set(TalonFXControlMode::PercentOutput, output);
    })
    .BeforeStarting([this, angle] {
      m_controller.Reset();
      m_controller.SetSetpoint(angle.value());
    })
    .Until([this, angle] {
      return
        std::abs(GetVelocity()) < PivotConstants::kVelocityThreshold.value()
        && units::math::abs(GetAngle() - angle) < TurretConstants::kPositionThreshold;
    }).AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}

frc2::CommandPtr Pivot::SetHomedCommand() {
  return RunOnce([this] { m_isHomed = true; });
}

void Pivot::SetMotor(double output) {
  m_mainMotor.Set(TalonFXControlMode::PercentOutput, output); 
}