package frc.robot.lib.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveController {
    public ChassisSpeeds transform(ControllerDriveInputs inputs, Pose3d robotPose);
}
