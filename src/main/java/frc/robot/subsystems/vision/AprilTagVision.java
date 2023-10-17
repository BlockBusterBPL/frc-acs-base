// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.util.VirtualSubsystem;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AprilTagVision extends VirtualSubsystem {
    private final AprilTagVisionIO io;
    private final AprilTagVisionIOInputsAutoLogged inputs;

    private final Consumer<VisionPose> consumer;

    private boolean enableVisionUpdates = true;
    private Alert enableVisionUpdatesAlert =
      new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);

    public AprilTagVision(AprilTagVisionIO io, Consumer<VisionPose> consumer) {
        this.io = io;
        this.inputs = new AprilTagVisionIOInputsAutoLogged();
        this.consumer = consumer;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("AprilTags", inputs);

        if (inputs.visionConnected && inputs.poseValid && enableVisionUpdates) {
            Pose3d pose = new Pose3d(
                new Translation3d(
                    inputs.position[0], 
                    inputs.position[1], 
                    inputs.position[2]
                    ), 
                new Rotation3d(
                    inputs.position[3], 
                    inputs.position[4], 
                    inputs.position[5]
                    )
            );
            Matrix<N3, N1> stddevs = VecBuilder.fill(
                inputs.stddevs[0], 
                inputs.stddevs[1], 
                inputs.stddevs[2]
            );

            consumer.accept(new VisionPose(pose, inputs.lastUpdateTimestamp, stddevs));
        }
    }

    public void setVisionUpdatesEnabled(boolean enabled) {
        enableVisionUpdates = enabled;
        enableVisionUpdatesAlert.set(!enabled);
    }
}
