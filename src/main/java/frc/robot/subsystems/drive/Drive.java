// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.swerve.SwerveSetpointGenerator;
import frc.robot.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.planners.DriveMotionPlanner;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private GyroIO gyroIO;
    private GryoIOInputsAutoLogged gyroInputs;

    private final SwerveModule[] mModules;
    private final int kFrontLeftID = 0;
    private final int kFrontRightID = 1;
    private final int kRearLeftID = 2;
    private final int kRearRightID = 3;

    private SwerveSetpointGenerator mSetpointGenerator;

    private Rotation2d mYawOffset;

    private final DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;
    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    private KinematicLimits mKinematicLimits = Constants.kFastKinematicLimits;

    private boolean isBrakeMode = false;
    private Timer lastMovementTimer = new Timer();

    public enum DriveControlState {
        OPEN_LOOP,
        VELOCITY_CONTROL,
        PATH_FOLLOWING
    }

    public Drive(GyroIO driveIO, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO, SwerveModuleIO rearLeftIO, SwerveModuleIO rearRightIO) {
        mModules = new SwerveModule[4];

        mModules[kFrontLeftID] = new SwerveModule(frontLeftIO, kFrontLeftID);
        mModules[kFrontRightID] = new SwerveModule(frontRightIO, kFrontRightID);
        mModules[kRearLeftID] = new SwerveModule(rearLeftIO, kRearLeftID);
        mModules[kRearRightID] = new SwerveModule(rearRightIO, kRearRightID);

        lastMovementTimer.reset();

        mMotionPlanner = new DriveMotionPlanner();
    }

    @Override
    public void periodic() {
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    }
}
