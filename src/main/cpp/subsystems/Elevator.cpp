// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
  ConfigMotor(&m_verticalMotor);
};

// This method will be called once per scheduler run
void Elevator::Periodic() {}

frc2::CommandPtr Elevator::DefaultControlCommand(std::function<double()> magnitude) {
  return Run([this, magnitude = std::move(magnitude)] {
    if (std::abs(magnitude()) > 0.05) m_verticalMotor.Set(TalonFXControlMode::PercentOutput, magnitude() * 0.6);
  });
}

void Elevator::ConfigMotor(WPI_TalonFX* motor) {
  motor->ConfigMotionCruiseVelocity(30000);
  motor->ConfigMotionAcceleration(8000);

  motor->SelectProfileSlot(0, 0);
  motor->Config_kP(0, VerticalElevatorConstants::kElevatorP);
  motor->Config_kI(0, VerticalElevatorConstants::kElevatorI);
  motor->Config_kD(0, VerticalElevatorConstants::kElevatorD);

  motor->ConfigNominalOutputForward(0);
  motor->ConfigNominalOutputReverse(0);
  motor->ConfigPeakOutputForward(1);
  motor->ConfigPeakOutputReverse(-1);

  motor->SetNeutralMode(NeutralMode::Brake);
  motor->SetSafetyEnabled(false);
  motor->SetInverted(false);

  motor->GetSensorCollection().SetIntegratedSensorPositionToAbsolute();
  motor->GetSensorCollection().SetIntegratedSensorPosition(0);

  motor->OverrideLimitSwitchesEnable(true); 
  motor->ConfigForwardSoftLimitEnable(false);
  motor->ConfigReverseSoftLimitEnable(false);
}; 

frc2::CommandPtr Elevator::MoveVertical(int direction) {
    double output = 0.25; 
    if (direction < 0) output *= -1;
    if (!(direction == 0)) m_verticalMotor.Set(TalonFXControlMode::PercentOutput, output);  
};

frc2::CommandPtr Elevator::ZeroVertical() {
    return Run([this] { m_verticalMotor.Set(TalonFXControlMode::PercentOutput, -0.2); })
    .Until([this] { return m_verticalMotor.IsRevLimitSwitchClosed(); })
    .AndThen([this] {
        m_verticalMotor.GetSensorCollection().SetIntegratedSensorPosition(0);
        m_verticalZeroed = true; 
    });
};

units::inch_t Elevator::GetDistance() {
  return units::inch_t(m_verticalMotor.GetSensorCollection().GetIntegratedSensorPosition() * VerticalElevatorConstants::kElevatorInchesPerTick);
};

void Elevator::SetDistance(WPI_TalonFX* motor, units::inch_t distance, double kElevatorInchesPerTick) {
    double ticks = distance.value() / kElevatorInchesPerTick;
    motor->Set(TalonFXControlMode::MotionMagic, ticks); 
}; 

frc2::CommandPtr Elevator::SetDistanceCommandVertical(units::inch_t distance) {
    return this->RunOnce(
        [this, distance] {
            if(m_verticalZeroed) SetDistance(&m_verticalMotor, distance, VerticalElevatorConstants::kElevatorInchesPerTick);
        }
    );
}

