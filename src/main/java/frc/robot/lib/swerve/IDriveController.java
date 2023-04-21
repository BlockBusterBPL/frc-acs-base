package frc.robot.lib.swerve;

import frc.robot.lib.geometry.Pose2d;

public interface IDriveController {
    ImprovedChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose);
}
