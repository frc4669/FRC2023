// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pivot.h"
#include <iostream>

Pivot::Pivot() { 
  m_mainMotor.ConfigMotionCruiseVelocity(7000);
  m_mainMotor.ConfigMotionAcceleration(3000);

  m_mainMotor.ConfigNominalOutputForward(0);
  m_mainMotor.ConfigNominalOutputReverse(0);
  m_mainMotor.ConfigPeakOutputForward(1);
  m_mainMotor.ConfigPeakOutputReverse(-1);

  m_mainMotor.SetNeutralMode(NeutralMode::Brake);
  m_mainMotor.SetSafetyEnabled(false);
  m_mainMotor.SetInverted(false);

  m_mainMotor.OverrideLimitSwitchesEnable(true); // Forward = pivoted out

  m_mainMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
}

void Pivot::Periodic() {
  frc::SmartDashboard::PutNumber("Pivot Angle (deg)", (GetAngle()).value());
}

units::degree_t Pivot::GetAngle() {
  return units::degree_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorPosition() * PivotConstants::kDegreesPerTick);
}

units::degrees_per_second_t Pivot::GetVelocity() {
  return units::degree_t(m_mainMotor.GetSensorCollection().GetIntegratedSensorVelocity() * 10 * PivotConstants::kDegreesPerTick) / 1_s;
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
    Run([this, angle] {
      units::degree_t currentAngle = GetAngle();
      frc::TrapezoidProfile<units::degrees>::State state = m_controller.GetSetpoint();
      
      units::volt_t output = units::volt_t(m_controller.Calculate(currentAngle));
      units::radian_t radAngle = units::radian_t(state.position + PivotConstants::kHorizontalOffset);
      units::volt_t ff = m_feedforward.Calculate(
        radAngle,
        state.velocity//,
        // (state.velocity - GetVelocity()) / 20_ms
      );
      // std::cout << "test\n";
      // std::cout << std::to_string(ff.value()) << "\n";
      // std::cout << std::to_string(radAngle.value()) << "\n";
      m_mainMotor.SetVoltage(ff + output);
    })
    .BeforeStarting([this, angle] {
      m_controller.Reset(GetAngle(), GetVelocity());
      m_controller.SetGoal(angle);
    })
    .Until([this, angle] {
      return
        units::math::abs(GetVelocity()) < PivotConstants::kVelocityThreshold
        && units::math::abs(GetAngle() - angle) < TurretConstants::kPositionThreshold;
    }).AndThen([this] { m_mainMotor.Set(TalonFXControlMode::PercentOutput, 0); }).WithTimeout(2_s),
    RunOnce([] {}), [this] { return m_isHomed; }
  );
}

frc2::CommandPtr Pivot::SetHomedCommand() {
  return RunOnce([this] { m_isHomed = true; });
}

void Pivot::SetHomed() {
  m_isHomed = true;
}