// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class FieldOrientedDriveController implements DriveController {

    @Override
    public ChassisSpeeds transform(ControllerDriveInputs inputs, Pose3d robotPose) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(inputs.getVelocity(4, 2), robotPose.getRotation().toRotation2d());
    }
}
