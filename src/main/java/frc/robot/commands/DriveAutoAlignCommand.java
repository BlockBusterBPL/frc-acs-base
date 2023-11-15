// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class DriveAutoAlignCommand extends CommandBase {
    private Drive drive;
    private Arm arm;
    private Optional<Pose2d> targetPoint;

    public DriveAutoAlignCommand(Drive drive, Arm arm) {
        addRequirements(drive, arm);

        this.drive = drive;
        this.arm = arm;
        targetPoint = Optional.empty();
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // get target point
        targetPoint = AutoAlignPointSelector.chooseTargetPoint(
            drive.getPose(), 
            arm.gameObjectIsCone() ? RequestedAlignment.AUTO_CONE : RequestedAlignment.AUTO_CUBE
        );

        if (targetPoint.isPresent()) {
            drive.setSnapYTheta(targetPoint.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
