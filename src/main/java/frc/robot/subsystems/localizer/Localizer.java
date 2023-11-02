// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.localizer;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.util.VirtualSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.localizer.LocalizerIOInputsAutoLogged;

/** Add your docs here. */
public class Localizer extends VirtualSubsystem {
    private final LocalizerIO io;
    private final LocalizerIOInputsAutoLogged inputs;

    private final Consumer<VisionPose> consumer;

    public final SendableChooser<Boolean> visionEnableChooser = new SendableChooser<>();

    private boolean enableVisionUpdates = true;
    private Alert enableVisionUpdatesAlert =
      new Alert("Vision updates are manually disabled.", AlertType.WARNING);

    public Localizer(LocalizerIO io, Consumer<VisionPose> consumer) {
        this.io = io;
        this.inputs = new LocalizerIOInputsAutoLogged();
        this.consumer = consumer;

        visionEnableChooser.setDefaultOption("Enabled", true);
        visionEnableChooser.addOption("Disabled", false);
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
