// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** Add your docs here. */
public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public double[] position = new double[6];
        public double[] stddevs = new double[3];
        public long targetsVisible = 0;
        public double lastUpdateTimestamp = 0.0;

        public boolean visionConnected = false;
        public boolean poseValid = false;
    }

    public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
