// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.subsystems.drive.Drive;

public class DriveWithController extends CommandBase {
    private final Drive drive;
    private final Supplier<ControllerDriveInputs> driveInputSupplier;
    private final Supplier<Boolean> slowModeSupplier;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public static final SendableChooser<Double> linearSpeedLimitChooser = new SendableChooser<>();

    public static final SendableChooser<Double> angularSpeedLimitChooser = new SendableChooser<>();

    static {
        linearSpeedLimitChooser.setDefaultOption("--Competition Mode--", 1.0);
        linearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        linearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        linearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
        angularSpeedLimitChooser.setDefaultOption("--Competition Mode--", 1.0);
        angularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        angularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        angularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    }

    /** Creates a new DefaultDriveCommand. */
    public DriveWithController(
            Drive drive,
            Supplier<ControllerDriveInputs> driveInputSupplier,
            Supplier<Boolean> slowModeSupplier) {
        addRequirements(drive);

        this.drive = drive;
        this.driveInputSupplier = driveInputSupplier;
        this.slowModeSupplier = slowModeSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var linearSpeedFactor = linearSpeedLimitChooser.getSelected();
        var angularSpeedFactor = angularSpeedLimitChooser.getSelected();
        if (slowModeSupplier.get()) {
            linearSpeedFactor *= 0.25;
            angularSpeedFactor *= 0.25;
        }

        var output = driveInputSupplier.get()
        .times(linearSpeedFactor, angularSpeedFactor)
        .getVelocityFieldOriented(
            Constants.kMaxVelocityMetersPerSecond, 
            Constants.kMaxAngularVelocityRadiansPerSecond, 
            drive.getPose().getRotation()
        );
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
