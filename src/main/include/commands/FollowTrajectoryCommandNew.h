// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowTrajectoryCommandNew
    : public frc2::CommandHelper<frc2::CommandBase, FollowTrajectoryCommandNew> {
 public:
  FollowTrajectoryCommandNew(Drivetrain* drivetrain, frc::Field2d* field, std::string path);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc::RamseteController m_ramseteController;
  frc::Timer m_timer;
  frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };
  frc::PIDController m_leftController { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };
  frc::PIDController m_rightController { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };

  units::second_t m_prevTime;

  Drivetrain* m_drivetrain;
};
