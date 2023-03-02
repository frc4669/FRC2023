// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pivot.h"

Pivot::Pivot() { 
  m_controlMotor.ConfigMotionCruiseVelocity(7000);
  m_controlMotor.ConfigMotionAcceleration(3000);

  m_controlMotor.SelectProfileSlot(0, 0);
  m_controlMotor.Config_kP(0, PivotConstants::kPivotP);
  m_controlMotor.Config_kI(0, PivotConstants::kPivotI);
  m_controlMotor.Config_kD(0, PivotConstants::kPivotD);

  //m_controlMotor.ConfigContinuousCurrentLimit();

  m_controlMotor.ConfigNominalOutputForward(0);
  m_controlMotor.ConfigNominalOutputReverse(0);
  m_controlMotor.ConfigPeakOutputForward(1);
  m_controlMotor.ConfigPeakOutputReverse(-1);

  m_controlMotor.SetNeutralMode(NeutralMode::Brake);
  m_controlMotor.SetSafetyEnabled(false);
  m_controlMotor.SetInverted(true);

  m_controlMotor.GetSensorCollection().SetQuadraturePosition(0);
};

void Pivot::SetAngle(units::degree_t angle) {
  double ticks = angle.value() / PivotConstants::kPivotDegreesPerTick;
  frc::SmartDashboard::PutNumber("Target Pivot Pos", ticks); 
  m_controlMotor.Set(TalonSRXControlMode::MotionMagic, ticks); 
}

// This method will be called once per scheduler run
void Pivot::Periodic() {
  frc::SmartDashboard::PutNumber("Current Pivot Pos", m_controlMotor.GetSensorCollection().GetQuadraturePosition()); 
}

frc2::CommandPtr Pivot::SetPosDownCommand() {
  return RunOnce(
    [this] {
      this->SetAngle(90_deg); 
    }
  );
}

frc2::CommandPtr Pivot::SetPosOutCommand() {
  return RunOnce(
    [this] {
      this->SetAngle(180_deg); 
    }
  );
}

frc2::CommandPtr Pivot::SetPosInCommand() {
  return RunOnce(
    [this] {
      this->SetAngle(0_deg); 
    }
  );
}