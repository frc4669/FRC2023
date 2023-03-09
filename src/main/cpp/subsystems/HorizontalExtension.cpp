// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/HorizontalExtension.h"

HorizontalExtension::HorizontalExtension() {
  ConfigMotor(&m_horizontalMotor);
};

// This method will be called once per scheduler run
void HorizontalExtension::Periodic() {}

void HorizontalExtension::ConfigMotor(WPI_TalonFX* motor) {
  motor->ConfigMotionCruiseVelocity(30000);
  motor->ConfigMotionAcceleration(8000);

  motor->SelectProfileSlot(0, 0);
  motor->Config_kP(0, HorizontalElevatorConstants::kElevatorP);
  motor->Config_kI(0, HorizontalElevatorConstants::kElevatorI);
  motor->Config_kD(0, HorizontalElevatorConstants::kElevatorD);

  motor->ConfigNominalOutputForward(0);
  motor->ConfigNominalOutputReverse(0);
  motor->ConfigPeakOutputForward(1);
  motor->ConfigPeakOutputReverse(-1);

  motor->SetNeutralMode(NeutralMode::Brake);
  motor->SetSafetyEnabled(false);
  motor->SetInverted(true);

  motor->GetSensorCollection().SetIntegratedSensorPositionToAbsolute();
  motor->GetSensorCollection().SetIntegratedSensorPosition(0);

  motor->OverrideLimitSwitchesEnable(true); 
  motor->ConfigForwardSoftLimitEnable(false);
  motor->ConfigReverseSoftLimitEnable(false);
}; 

frc2::CommandPtr HorizontalExtension::DefaultControlCommand(std::function<double()> magnitude) {
  return Run([this, magnitude = std::move(magnitude)] {
    if (std::abs(magnitude()) > 0.05) m_horizontalMotor.Set(TalonFXControlMode::PercentOutput, magnitude() * 0.6);
  });
}

frc2::CommandPtr HorizontalExtension::MoveHorizontal(int direction) {
    double output = 0.25; 
    if (direction < 0) output *= -1;
    if (!(direction == 0)) m_horizontalMotor.Set(TalonFXControlMode::PercentOutput, output);  
};



frc2::CommandPtr HorizontalExtension::ZeroHorizontal() {
    return Run([this] { m_horizontalMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_horizontalMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
        m_horizontalMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
        m_horizontalZeroed = true; 
    });
}; 

units::inch_t HorizontalExtension::GetDistance(WPI_TalonFX* motor, double kHorizontalExtensionInchesPerTick) {
  return units::inch_t(motor->GetSensorCollection().GetIntegratedSensorPosition() * kHorizontalExtensionInchesPerTick);
};

void HorizontalExtension::SetDistance(WPI_TalonFX* motor, units::inch_t distance, double kHorizontalExtensionInchesPerTick) {
    double ticks = distance.value() / kHorizontalExtensionInchesPerTick;
    motor->Set(TalonFXControlMode::MotionMagic, ticks); 
}; 

frc2::CommandPtr HorizontalExtension::SetDistanceCommandHorizontal(units::inch_t distance) {
    return this->RunOnce(
        [this, distance] {
            if (m_horizontalZeroed) SetDistance(&m_horizontalMotor, distance, HorizontalElevatorConstants::kElevatorInchesPerTick);
        }
    );
}