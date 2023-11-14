// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.Utility;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.drive.SwerveHeadingController;
import frc.robot.lib.drive.SwerveHeadingController.HeadingControllerState;
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;

public class DriveWithController extends CommandBase {
    private final Drive drive;
    private final Supplier<ControllerDriveInputs> driveInputSupplier;
    private final Supplier<Boolean> slowModeSupplier;

    private final TimeDelayedBoolean mShouldMaintainHeading = new TimeDelayedBoolean();
    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    private Optional<Double> mHeadingGoal = Optional.empty();

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
        mShouldMaintainHeading.update(false, 0);
        mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
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

        var controllerInputs = driveInputSupplier.get()
        .times(linearSpeedFactor, angularSpeedFactor);

        boolean drive_turning = !Util.epsilonEquals(controllerInputs.getRotation(), 0);
        boolean drive_translating = Utility.getSpeedAsScalar(drive.getMeasuredSpeeds()) >= 0.1;

        if (mShouldMaintainHeading.update(!drive_turning && drive_translating, 0.2)) {
            mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
            mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
        } else {
            mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
            mHeadingGoal = Optional.of(drive.getFieldOrientation().getZ());
        }

        if (mSwerveHeadingController.getHeadingControllerState() != HeadingControllerState.OFF) {
            controllerInputs.setRotation(mSwerveHeadingController.update(drive.getFieldOrientation().getZ())); //TODO: degrees
        }

        var speedsFromController = controllerInputs.getVelocityFieldOriented(
            Constants.kMaxVelocityMetersPerSecond, 
            Constants.kMaxAngularVelocityRadiansPerSecond, 
            // drive.getPose().getRotation()
            new Rotation2d()
        );

        drive.swerveDrive(speedsFromController);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        mShouldMaintainHeading.update(false, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
