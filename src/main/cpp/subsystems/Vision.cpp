// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision(frc::Field2d* field, Drivetrain* drivetrain) : m_field(field), m_drivetrain(drivetrain) {};

// This method will be called once per scheduler run
void Vision::Periodic() {
    frc::Transform3d tagTransform = GetFirstTagPose();
    if (tagTransform.X() == 0_m && tagTransform.Y() == 0_m && tagTransform.Z() == 0_m) return;

    frc::Pose2d robotPose = RobotPose(kTagPose, tagTransform);

    m_field->GetObject("visionPose")->SetPose(robotPose);
    m_field->GetObject("tag")->SetPose(kTagPose.ToPose2d());
}

frc::Transform3d Vision::GetFirstTagPose() {
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("SmartDashboard/VisionServer/Tags");
    std::vector<std::string> tagIds = table->GetSubTables();

    if (tagIds.size() == 0) return frc::Transform3d();

    auto tag = table->GetSubTable(tagIds[0]);
    auto rot = tag->GetSubTable("Rot");
    auto trans = tag->GetSubTable("Trans");

    frc::Rotation3d rvec(units::degree_t(rot->GetNumber("Y", 0) * 180 / 3.141592), -units::degree_t(rot->GetNumber("X", 0) * 180 / 3.141592) + 90_deg, units::degree_t(rot->GetNumber("Z", 0) * 180 / 3.141592));
    frc::Translation3d tvec(units::inch_t(trans->GetNumber("Z", 0)), units::inch_t(trans->GetNumber("X", 0)), units::inch_t(trans->GetNumber("Y", 0)));

    frc::Transform3d tagPose(tvec, frc::Rotation3d(rvec));
    return tagPose;
}

frc::Pose2d Vision::RobotPose(frc::Pose3d tagPose, frc::Transform3d relative) {
    frc::Pose3d cameraPose = tagPose.TransformBy(relative);
    frc::Pose3d robotPose = cameraPose.TransformBy(kCameraToRobot);
    return frc::Pose2d(robotPose.ToPose2d().Translation(), frc::Rotation2d(-m_drivetrain->GetYaw()));
}

//////////////////////
// START OBJ DETECT //
//////////////////////
Vision::ObjDetectResults ObjDetectGetResults() {
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault(); 
    auto table = inst.GetTable("SmartDashboard/VisionServer/Objects"); 

    if (table->GetBoolean("HasTargets", false) == false) {
        return Vision::ObjDetectResults(); 
    }; 

    Vision::ObjDetectResults results;

    auto cones = table->GetSubTable("Cones");

    std::string yawStringData = cones->GetString("Yaw_Values", ""); 

    // Tokenize & Interate throught all the data 
    int start = 0, end = -1; 
    do {
        start = end + 1; 
        end = yawStringData.find(',', start); 
        double yaw = std::stod(yawStringData.substr(start, end - start)); 

        Vision::ObjDetectTarget target = {
            .yaw = units::degree_t(yaw)
        };

        results.targets.push_back(target); 

    } while (end != -1); 

    return results; 
}

Vision::ObjDetectTarget ObjDetectGetBestTarget(Vision::ObjDetectResults results) {
    //TO DO
}
////////////////////
// END OBJ DETECT //
////////////////////