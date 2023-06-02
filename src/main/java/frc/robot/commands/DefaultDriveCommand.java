// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.drive.DriveController;
import frc.robot.subsystems.drive.Drive;

public class DefaultDriveCommand extends CommandBase {
    private final Drive drive;
    private final Supplier<ControllerDriveInputs> driveInputSupplier;
    private final Supplier<DriveController> driveControllerSupplier;

    // private final Supplier<ControllerDriveInputs> driveInputSupplier;
    // private final Supplier<DriveController> driveControllerSupplier;
    // private final Supplier<Boolean> slowModeSupplier;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    private static final LoggedDashboardChooser<Double> linearSpeedLimitChooser = new LoggedDashboardChooser<>(
            "Linear Speed Limit");

    private static final LoggedDashboardChooser<Double> angularSpeedLimitChooser = new LoggedDashboardChooser<>(
            "Angular Speed Limit");

    static {

    }

    /** Creates a new DefaultDriveCommand. */
    public DefaultDriveCommand(
            Drive drive,
            Supplier<ControllerDriveInputs> driveInputSupplier,
            Supplier<DriveController> driveControllerSupplier
        ) {
        addRequirements(drive);

        this.drive = drive;
        this.driveInputSupplier = driveInputSupplier;
        this.driveControllerSupplier = driveControllerSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var inputs = driveInputSupplier.get();
        var controller = driveControllerSupplier.get();
        var output = controller.transform(inputs, new Pose3d(drive.getPose()));
        drive.swerveDrive(output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
