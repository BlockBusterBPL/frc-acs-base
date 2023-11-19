// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
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
    private final Supplier<Boolean> disableFieldOrient;
    private final Supplier<Boolean> snapClosestCardinal;
    private final Supplier<Boolean> snapOppositeCardinal;

    private boolean snapOppositeFirstRun = true;
    private double snapOppositeReferenceAngle = 0.0;

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
            Supplier<Boolean> slowModeSupplier,
            Supplier<Boolean> disableFieldOrient,
            Supplier<Boolean> snapClosestCardinal,
            Supplier<Boolean> snapOppositeCardinal
        ) {
        addRequirements(drive);

        this.drive = drive;
        this.driveInputSupplier = driveInputSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.disableFieldOrient = disableFieldOrient;
        this.snapClosestCardinal = snapClosestCardinal;
        this.snapOppositeCardinal = snapOppositeCardinal;
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
            linearSpeedFactor *= 0.375;
            angularSpeedFactor *= 0.375;
        }

        var controllerInputs = driveInputSupplier.get()
        .times(linearSpeedFactor, angularSpeedFactor);

        boolean drive_turning = !Util.epsilonEquals(controllerInputs.getRotation(), 0);
        boolean drive_translating = Utility.getSpeedAsScalar(drive.getMeasuredSpeeds()) >= 0.1;

        boolean shouldSnapClosestCardinal = snapClosestCardinal.get();
        boolean shouldSnapOppositeCardinal = snapOppositeCardinal.get();
        boolean autoMaintain = mShouldMaintainHeading.update(!drive_turning && drive_translating && !shouldSnapClosestCardinal && !shouldSnapOppositeCardinal, 0.2);

        double robotAngleDegrees = drive.getPose().getRotation().getDegrees();

        if (shouldSnapOppositeCardinal) {
            if (snapOppositeFirstRun) {
                snapOppositeReferenceAngle = robotAngleDegrees;
                snapOppositeFirstRun = false;
            }
            double closestCardinal = getClosestCardinal(snapOppositeReferenceAngle);
            double oppositeCardinal = (closestCardinal == 0.0 ? -180.0 : 0.0);
            // System.out.println("closest cardinal: " + closestCardinal + " opposite cardinal: " + oppositeCardinal);
            mHeadingGoal = Optional.of(oppositeCardinal);
        } else {
            snapOppositeFirstRun = true;
        }

        if (shouldSnapClosestCardinal && !shouldSnapOppositeCardinal) {
            double closestCardinal = getClosestCardinal(drive.getPose().getRotation().getDegrees());
            mHeadingGoal = Optional.of(closestCardinal);
        } else if (!autoMaintain && !shouldSnapOppositeCardinal) {
            mHeadingGoal = Optional.of(drive.getPose().getRotation().getDegrees());
        }

        if (autoMaintain || shouldSnapClosestCardinal || shouldSnapOppositeCardinal) {
            mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
            if (mSwerveHeadingController.getAbsError() <= Constants.kSwerveHeadingControllerMaintainThreshold) {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
            } else {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.SNAP);
            }
        } else {
            mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        }

        // if (snapClosestCardinal.get()) {
        //     double closestCardinalHeading = 0;
        //     mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.SNAP);
        //     mSwerveHeadingController.setGoal(closestCardinalHeading);            
        // } else if (mShouldMaintainHeading.update(!drive_turning && drive_translating, 0.2)) {
        //     mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
        //     mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
        // } else {
        //     mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        //     mHeadingGoal = Optional.of(drive.getFieldOrientation().getZ());
        // }

        if (mSwerveHeadingController.getHeadingControllerState() != HeadingControllerState.OFF) {
            controllerInputs.setRotation(mSwerveHeadingController.update(drive.getPose().getRotation().getDegrees()));
        }

        var speedsFromController = controllerInputs.getVelocityFieldOriented(
            Constants.kMaxVelocityMetersPerSecond, 
            Constants.kMaxAngularVelocityRadiansPerSecond, 
            ( disableFieldOrient.get() ? new Rotation2d() : drive.getPose().getRotation() )
        );

        drive.setVelocityClosedLoop(speedsFromController);
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

    private double getClosestCardinal(double robotAngle) {
        return Math.round(robotAngle/180.0) * 180;
    }
}
