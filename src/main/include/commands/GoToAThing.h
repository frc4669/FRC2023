// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"

#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Timer.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GoToAThing
    : public frc2::CommandHelper<frc2::CommandBase, GoToAThing> {
 public:
  GoToAThing(Drivetrain* drivetrain, Vision* vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  // sub system requiremetns  
  Drivetrain* m_drivetrain; 
  Vision* m_vision; 

  //linear feed back 
  frc::PIDController m_leftPID { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd }; 
  frc::PIDController m_rightPId {DriveConstants::kp, DriveConstants::ki, DriveConstants::kd }; 

  //non linear feed back control 
  frc::RamseteController m_ramsete;
  frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };

  //feed forward 
  frc::SimpleMotorFeedforward<units::meters> m_feedforward { DriveConstants::ks, DriveConstants::kv, DriveConstants::ka }; 

  // tracking 
  frc::Timer m_timer; 
  units::second_t m_previousTime; 
  frc::DifferentialDriveWheelSpeeds m_previousSpeed; 

  // settings 
  units::meters_per_second_t kDESIRED_VEL = 1_mps;  
};
