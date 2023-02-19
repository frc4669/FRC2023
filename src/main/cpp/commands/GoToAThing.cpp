// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GoToAThing.h"

GoToAThing::GoToAThing(Drivetrain* drivetrain, Vision* vision) 
:m_drivetrain(drivetrain), m_vision(vision) 
{
  AddRequirements({ drivetrain, vision }); 
}

// Called when the command is initially scheduled.
void GoToAThing::Initialize() {
  m_previousTime = units::second_t(-1);

  m_previousSpeed = m_kinematics.ToWheelSpeeds(frc::ChassisSpeeds()); 

  m_leftPID.Reset(); 
  m_rightPId.Reset(); 

  m_timer.Reset(); 
  m_timer.Start();  
}

// Called repeatedly when this Command is scheduled to run
void GoToAThing::Execute() {
  units::second_t currentTime = m_timer.Get(); 
  units::second_t dt = currentTime - m_previousTime;
  m_previousTime = currentTime; 

  // get results
  Vision::ObjDetectResults results; 
  
  if (!results.hasTargets) {
    m_drivetrain->TankDriveVolts(0_V, 0_V); 
    m_previousSpeed = frc::DifferentialDriveWheelSpeeds(); 
    return; 
  }

  // get target 

  frc::Rotation2d rotationToTarget(results.targets[0].yaw); 

  units::meter_t distance = 1_m; // keep it one m cuz i dont wanna do complex vision math yet-

  // get current physical params 
  frc::DifferentialDriveWheelSpeeds currentSpeed = m_drivetrain->GetWheelSpeeds(); 

  // angular velocity 
  units::second_t timeToTarget = distance / kDESIRED_VEL; 
  units::radians_per_second_t angularVelocity = rotationToTarget.Radians() / timeToTarget; 

  frc::DifferentialDriveWheelSpeeds targetSpeed = m_kinematics.ToWheelSpeeds(
    m_ramsete.Calculate(
      frc::Pose2d(), 
      frc::Pose2d(
        units::meter_t(rotationToTarget.Cos()), 
        units::meter_t(rotationToTarget.Sin()), 
        frc::Rotation2d()
      ), 
      kDESIRED_VEL, 
      angularVelocity
    )
  ); 

  units::volt_t leftFF = m_feedforward.Calculate(targetSpeed.left, (targetSpeed.left - m_previousSpeed.left) / dt); 
  units::volt_t rightFF = m_feedforward.Calculate(targetSpeed.right, (targetSpeed.right - m_previousSpeed.right) / dt); 

  units::volt_t leftOut = units::volt_t(m_leftPID.Calculate(currentSpeed.left.value(), currentSpeed.right.value())) + leftFF; 
  units::volt_t rightOut = units::volt_t(m_rightPId.Calculate(currentSpeed.right.value(), targetSpeed.right.value())) + rightFF; 

  m_previousSpeed = targetSpeed;

  m_drivetrain->TankDriveVolts(leftOut, rightOut); 

}

// Called once the command ends or is interrupted.
void GoToAThing::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToAThing::IsFinished() {
  return false;
}
