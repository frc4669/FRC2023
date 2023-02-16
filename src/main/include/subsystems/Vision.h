// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <units/constants.h>
#include <units/angle.h>
#include <frc/smartdashboard/Field2d.h>

#include "subsystems/Drivetrain.h"

class Vision : public frc2::SubsystemBase {
 public:
  Vision(frc::Field2d* field, Drivetrain* drivetrain);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc::Transform3d GetFirstTagPose();

  frc::Transform3d kCameraToRobot {
    frc::Translation3d { -2.5_in, -13_in, -15_in },
    frc::Rotation3d { 0_deg, -30_deg, 0_deg }
  };

  frc::Pose3d kTagPose {
    2.25_m, 0_m, 0.75_m,
    frc::Rotation3d { 0_deg, 0_deg, 180_deg }
  };

  frc::Pose2d RobotPose(frc::Pose3d tagPose, frc::Transform3d relative);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::Field2d* m_field;
  Drivetrain* m_drivetrain;
};
