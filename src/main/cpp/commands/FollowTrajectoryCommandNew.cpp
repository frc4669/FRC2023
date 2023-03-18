// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowTrajectoryCommandNew.h"

FollowTrajectoryCommandNew::FollowTrajectoryCommandNew(
  Drivetrain* drivetrain,
  frc::Field2d* field,
  std::string path,
  bool reverse,
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel
) : m_drivetrain(drivetrain), m_field(field)
{
  AddRequirements({ drivetrain });

  pathplanner::PathPlannerTrajectory PPtrajectory = pathplanner::PathPlanner::loadPath(path, maxSpeed, maxAccel, reverse);
  m_trajectory = PPtrajectory.asWPILibTrajectory();
}

// Called when the command is initially scheduled.
void FollowTrajectoryCommandNew::Initialize() {
  m_prevTime = -1_s;
  m_finished = false;

  m_field->GetObject("trajectory")->SetTrajectory(m_trajectory);
  m_drivetrain->ResetOdometry(m_trajectory.InitialPose(), m_trajectory.InitialPose().Rotation());

  m_leftController.Reset();
  m_rightController.Reset();

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowTrajectoryCommandNew::Execute() {
  units::second_t currentTime = m_timer.Get();
  units::second_t delta = currentTime - m_prevTime;
  m_prevTime = currentTime;

  if (m_trajectory.TotalTime() < currentTime) { // REVIEW
    m_finished = true;
    return;
  }

  frc::DifferentialDriveWheelSpeeds currentSpeed = m_drivetrain->GetWheelSpeeds();
  
  frc::ChassisSpeeds outputChassisSpeed = m_ramseteController.Calculate(m_drivetrain->OdometryPose(), m_trajectory.Sample(currentTime));
  frc::DifferentialDriveWheelSpeeds outputSpeed = m_kinematics.ToWheelSpeeds(outputChassisSpeed);

  units::volt_t leftFF = m_feedforward.Calculate(outputSpeed.left, (outputSpeed.left - currentSpeed.left) / delta);
  units::volt_t rightFF = m_feedforward.Calculate(outputSpeed.right, (outputSpeed.right - currentSpeed.right) / delta);

  units::volt_t leftOut = units::volt_t(m_leftController.Calculate(currentSpeed.left.value(), outputSpeed.left.value()));
  units::volt_t rightOut = units::volt_t(m_rightController.Calculate(currentSpeed.right.value(), outputSpeed.right.value()));

  m_drivetrain->TankDriveVolts(leftOut + leftFF, rightOut + rightFF);
}

// Called once the command ends or is interrupted.
void FollowTrajectoryCommandNew::End(bool interrupted) {
  m_drivetrain->TankDriveVolts(0_V, 0_V);
}

// Returns true when the command should end.
bool FollowTrajectoryCommandNew::IsFinished() {
  return m_finished;
}
