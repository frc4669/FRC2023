// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowTrajectoryCommandNew.h"

FollowTrajectoryCommandNew::FollowTrajectoryCommandNew(Drivetrain* drivetrain, frc::Field2d* field, std::string path) 
{
  AddRequirements({ drivetrain });
}

// Called when the command is initially scheduled.
void FollowTrajectoryCommandNew::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FollowTrajectoryCommandNew::Execute() {}

// Called once the command ends or is interrupted.
void FollowTrajectoryCommandNew::End(bool interrupted) {}

// Returns true when the command should end.
bool FollowTrajectoryCommandNew::IsFinished() {
  return false;
}
