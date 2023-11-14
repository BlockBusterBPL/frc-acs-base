// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class DriveResetGyroCommand extends CommandBase {
    private final Drive drive;
    
    public DriveResetGyroCommand(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.reseedRotation();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
