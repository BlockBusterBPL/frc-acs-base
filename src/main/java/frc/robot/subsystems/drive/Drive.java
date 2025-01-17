// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotStateTracker;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.lib.Utility;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.drive.AutoAlignMotionPlanner;
import frc.robot.lib.drive.SwerveSetpoint;
import frc.robot.lib.drive.SwerveSetpointGenerator;
import frc.robot.lib.drive.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.subsystems.localizer.VisionPose;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private static final double kCoastThresholdMetersPerSec = 0.05; // Need to be under this to switch to coast when
                                                                   // disabling
    private static final double kCoastThresholdSecs = 6.0; // Need to be under the above speed for this length of time to
                                                          // switch to coast

    public enum DriveControlState {
        OPEN_LOOP("OL Velocity"),
        VELOCITY_CONTROL("CL Velocity"),
        PATH_FOLLOWING("CL Pathfinding"),
        AUTO_ALIGN("CL Auto Align"),
        AUTO_ALIGN_Y_THETA("CL Auto Align Y Theta"),
        X_MODE("CL X Mode");

        public String title;

        DriveControlState(String title) {
            this.title = title;
        }
    }

    private DriveControlState mControlState = DriveControlState.VELOCITY_CONTROL;
    private boolean mAllowDriveAssists = true;
    private AutoAlignMotionPlanner mAutoAlignPlanner = new AutoAlignMotionPlanner();
    private Pose2d mTargetPoint = new Pose2d();
    private KinematicLimits mKinematicLimits = Constants.kUncappedKinematicLimits;

    public static final SwerveModuleState[] kXModeStates = {
        new SwerveModuleState(0, Rotation2d.fromRotations(-0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(-0.125))
    };

    private GyroIO mGyroIO;
    private GyroIOInputsAutoLogged mGyroInputs;
    private boolean mIgnoreGyro = false;

    private final Alert mAlertGyroNotConnected = new Alert(
            "Gyro not connected! " + (Constants.getRobot() == RobotType.ROBOT_SIMBOT ? "This warning is expected when using the SimBot." : ""),
            AlertType.ERROR);

    private final Alert mAlertGyroManualFail = new Alert(
            "Gyro inputs manually ignored by driver.", 
            AlertType.WARNING);

    private final Alert mAlertUsingDeltaIntegration = new Alert(
            "Pose angle reverted to wheel delta integration mode, please monitor robot pose. Using autonomous is NOT RECCOMENDED!",
            AlertType.ERROR);

    private final Alert mAlertCoastModeEnabled = new Alert(
            "Swerve Drive Coast Mode Enabled.",
            AlertType.INFO);

    private final Alert mAlertSteerNeutralMode = new Alert(
            "Swerve Steering Neutral Mode Enabled! Performance may be reduced, as driving in this mode is not intended.",
            AlertType.WARNING);
    
    private final Alert mAlertOffsetsNotSafe = new Alert(
            "Swerve offset safety not enabled! Module offsets could be updated by accident", 
            AlertType.WARNING);

    private final SwerveModule[] mModules;
    private final int kFrontLeftID = 0;
    private final int kFrontRightID = 1;
    private final int kRearLeftID = 2;
    private final int kRearRightID = 3;

    SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Constants.kWheelPositions);
    SwerveSetpointGenerator mGenerator = new SwerveSetpointGenerator(mKinematics);

    private ChassisSpeeds mSetpoint = new ChassisSpeeds();
    private ChassisSpeeds mMeasuredSpeeds = new ChassisSpeeds();

    private SwerveSetpoint mLastSetpoint = SwerveSetpoint.FOUR_WHEEL_IDENTITY;

    private boolean mIsBrakeMode = false;
    private Timer mLastMovementTimer = new Timer();

    private double[] mLastModuleDistancesMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private SwerveModulePosition[] mLastSwervePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d mLastGyroYaw = new Rotation2d();
    private Pose2d mLastRobotPose = new Pose2d();
    private Rotation3d mLastRotation3d = new Rotation3d();

    private SwerveDrivePoseEstimator mPoseEstimator;

    private Rotation2d mAutonRotationTarget;

    private boolean mAllowUpdateEncoders = false;

    public SendableChooser<Boolean> mEncoderUpdateChooser = new SendableChooser<>();
    private double[] mEncoderOffsetCache;

    public Drive(GyroIO gyroIO, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO, SwerveModuleIO rearLeftIO,
            SwerveModuleIO rearRightIO) {
        mGyroIO = gyroIO;
        mGyroInputs = new GyroIOInputsAutoLogged();
        mModules = new SwerveModule[4];

        mModules[kFrontLeftID] = new SwerveModule(frontLeftIO, kFrontLeftID);
        mModules[kFrontRightID] = new SwerveModule(frontRightIO, kFrontRightID);
        mModules[kRearLeftID] = new SwerveModule(rearLeftIO, kRearLeftID);
        mModules[kRearRightID] = new SwerveModule(rearRightIO, kRearRightID);

        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, mLastGyroYaw, mLastSwervePositions, getPose());

        mAutonRotationTarget = new Rotation2d();

        mEncoderUpdateChooser.setDefaultOption("Disabled (Safe)", false);
        mEncoderUpdateChooser.addOption("ENABLED (UNSAFE)", true);
        mEncoderOffsetCache = new double[4];

        mLastMovementTimer.reset();
    }

    @Override
    public void periodic() {
        // Update subsystem inputs - this should always be the first thing in periodic()
        mGyroIO.updateInputs(mGyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", mGyroInputs);

        for (SwerveModule swerveModule : mModules) {
            swerveModule.periodic();
        }

        // Update battery simulation in sim mode
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            for (SwerveModule module : mModules) {
                simCurrent += module.getTotalCurrent();
            }
            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        if (DriverStation.isDisabled()) {
            for (var module : mModules) {
                module.stop();
            }
            // Make sure to clear reported swerve setpoints
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
        } else {
            // Generate swerve setpoint

            SwerveModuleState[] setpointStates = new SwerveModuleState[4];

            Optional<ChassisSpeeds> driveSetpointOverride = Optional.empty();

            switch (mControlState) {
                case PATH_FOLLOWING:
                    setKinematicLimits(Constants.kFastKinematicLimits);
                    driveSetpointOverride = updatePathFollower();
                    Logger.getInstance().recordOutput("Drive/AutoAlign/LastPoseTarget", new Pose2d());
                    break;
                case OPEN_LOOP:
                case VELOCITY_CONTROL:
                    setKinematicLimits(Constants.kTeleopKinematicLimits);
                    Logger.getInstance().recordOutput("Drive/AutoAlign/LastPoseTarget", new Pose2d());
                    break;
                case AUTO_ALIGN:
                case AUTO_ALIGN_Y_THETA:
                    setKinematicLimits(Constants.kUncappedKinematicLimits);
                    driveSetpointOverride = updateAutoAlign();
                    Logger.getInstance().recordOutput("Drive/AutoAlign/LastPoseTarget", mTargetPoint);
                    break;
                case X_MODE:
                    setKinematicLimits(Constants.kUncappedKinematicLimits);
                    Logger.getInstance().recordOutput("Drive/AutoAlign/LastPoseTarget", new Pose2d());
                default:
                    Logger.getInstance().recordOutput("Drive/AutoAlign/LastPoseTarget", new Pose2d());
                    break;
            }

            if (driveSetpointOverride.isPresent() && mAllowDriveAssists) {
                setSetpoint(driveSetpointOverride.get());
            }

            if (mControlState == DriveControlState.X_MODE) {
                SwerveSetpoint generatedSetpoint = mGenerator.generateSetpointForXMode(mKinematicLimits, mLastSetpoint, Constants.loopPeriodSecs);
                mLastSetpoint = generatedSetpoint;
                setpointStates = generatedSetpoint.mModuleStates;
            } else {
                SwerveSetpoint generatedSetpoint = mGenerator.generateSetpoint(mKinematicLimits, mLastSetpoint,
                    mSetpoint, Constants.loopPeriodSecs);
                mLastSetpoint = generatedSetpoint;
                setpointStates = generatedSetpoint.mModuleStates;
            }

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                if (mControlState == DriveControlState.OPEN_LOOP) {
                    optimizedStates[i] = mModules[i].setStateOpenLoop(setpointStates[i]);
                } else {
                    optimizedStates[i] = mModules[i].setStateClosedLoop(setpointStates[i]);
                }
            }

            RobotStateTracker.getInstance().setCurrentRobotPose(mLastRobotPose);
            RobotStateTracker.getInstance().setCurrentRobotPosition(mLastRobotPose.getTranslation());
            RobotStateTracker.getInstance().setCurrentRobotVelocity(mMeasuredSpeeds);
            RobotStateTracker.getInstance().setAutoAlignActive(mControlState == DriveControlState.AUTO_ALIGN || mControlState == DriveControlState.AUTO_ALIGN_Y_THETA);
            RobotStateTracker.getInstance().setAutoAlignComplete(autoAlignAtTarget());

            // Log setpoint states
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);

            Logger.getInstance().recordOutput("Drive/ControlState", mControlState.title);
            Logger.getInstance().recordOutput("Drive/KinematicLimits", getKinematicLimitsTitle());
            Logger.getInstance().recordOutput("Drive/BrakeMode", mIsBrakeMode);
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = mModules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (mModules[i].getDistance() - mLastModuleDistancesMeters[i]),
                    mModules[i].getAngle());
            mLastModuleDistancesMeters[i] = mModules[i].getDistance();
        }

        // Calculate robot velocity from measured chassis speeds
        var inverseSpeeds = mKinematics.toChassisSpeeds(measuredStates);
        mMeasuredSpeeds = inverseSpeeds;

        // Update gyro angle
        if (!shouldRevertToDeltaIntegration()) {
            mLastGyroYaw = Rotation2d.fromRotations(mGyroInputs.yawAngleRotations);
        } else {
            // estimate rotation angle from wheel deltas when gyro is not connected
            mLastGyroYaw = mLastGyroYaw
                    .plus(new Rotation2d(inverseSpeeds.omegaRadiansPerSecond).times(Constants.loopPeriodSecs));
        }

        for (int i = 0; i < mModules.length; i++) {
            mLastSwervePositions[i] = mModules[i].getPosition();
        }

        // Update odometry
        mLastRobotPose = mPoseEstimator.update(mLastGyroYaw, mLastSwervePositions);

        // poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Log 3D odometry pose
        var robotTranslation3d = new Translation3d(
                mLastRobotPose.getX(),
                mLastRobotPose.getY(),
                0);

        mLastRotation3d = new Rotation3d(
                mGyroInputs.pitchAngleRotations * 2 * Math.PI,
                mGyroInputs.rollAngleRotations * 2 * Math.PI,
                mLastGyroYaw.getRadians());

        var robotPose3d = new Pose3d(robotTranslation3d, mLastRotation3d);

        Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

        // Update brake mode
        boolean stillMoving = false;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(mModules[i].getVelocity()) > kCoastThresholdMetersPerSec) {
                stillMoving = true;
            }
        }
        if (stillMoving)
            mLastMovementTimer.reset();
        if (DriverStation.isEnabled()) {
            if (!mIsBrakeMode) {
                mIsBrakeMode = true;
                for (var module : mModules) {
                    module.setDriveBrakeMode(true);
                }
            }
        } else {
            if (mIsBrakeMode && mLastMovementTimer.hasElapsed(kCoastThresholdSecs)) {
                mIsBrakeMode = false;
                for (var module : mModules) {
                    module.setDriveBrakeMode(false);
                }
            }
        }

        mAllowUpdateEncoders = mEncoderUpdateChooser.getSelected();

        // Run alert checks
        mAlertGyroNotConnected.set(!mGyroInputs.connected);
        mAlertGyroManualFail.set(mIgnoreGyro);
        mAlertUsingDeltaIntegration.set(shouldRevertToDeltaIntegration());
        mAlertCoastModeEnabled.set(!mIsBrakeMode);
        mAlertSteerNeutralMode.set(mModules[0].getSteerNeutralMode());
        mAlertOffsetsNotSafe.set(mAllowUpdateEncoders);
    }

    public void setVelocityClosedLoop(ChassisSpeeds speeds) {
        mControlState = DriveControlState.VELOCITY_CONTROL;
        mSetpoint = speeds;
    }

    public void setVelocityOpenLoop(ChassisSpeeds speeds) {
        mControlState = DriveControlState.OPEN_LOOP;
        mSetpoint = speeds;
    }

    public void setSnapToPoint(Pose2d targetPoint) {
        if (mAutoAlignPlanner != null) {
            if (mControlState != DriveControlState.AUTO_ALIGN) {
                mAutoAlignPlanner.reset();
                mControlState = DriveControlState.AUTO_ALIGN;
            }
            mTargetPoint = targetPoint;
            mAutoAlignPlanner.setTargetPoint(mTargetPoint);
        }
    }

    public void setSnapYTheta(Pose2d targetPoint) {
        if (mAutoAlignPlanner != null) {
            if (mControlState != DriveControlState.AUTO_ALIGN_Y_THETA) {
                mAutoAlignPlanner.reset();
                mControlState = DriveControlState.AUTO_ALIGN_Y_THETA;
            }
            mTargetPoint = targetPoint;
            mAutoAlignPlanner.setTargetPoint(mTargetPoint);
        }
    }

    private void setSetpoint(ChassisSpeeds speeds) {
        mSetpoint = speeds;
    }

    public Pose2d getTargetPoint() {
        return mTargetPoint;
    }

    public void reseedRotation() {
        mPoseEstimator.resetPosition(mLastGyroYaw, mLastSwervePositions, new Pose2d(mLastRobotPose.getTranslation(), new Rotation2d()));
    }
    
    public void stop() {
        setVelocityClosedLoop(new ChassisSpeeds());
    }

    public void setXMode() {
        mControlState = DriveControlState.X_MODE;
    }

    public Pose2d getPose() {
        return mLastRobotPose;
    }

    public Rotation3d getGyroAngle() {
        return mLastRotation3d;
    }

    public Rotation3d getFieldOrientation() {
        return mLastRotation3d.minus(new Rotation3d(0, 0, getGyroAngle().getZ()));
    }

    public Rotation2d getAutonRotationTarget() {
        return mAutonRotationTarget;
    }

    public void setAutonRotationTarget(Rotation2d autonTarget) {
        this.mAutonRotationTarget = autonTarget;
    }

    public void addVisionPose(VisionPose pose) {
        mPoseEstimator.addVisionMeasurement(pose.pose.toPose2d(), pose.timestampSeconds, pose.stddevs);
    }

    public boolean updateEncoderOffset(int module, double newOffset) {
        if (mAllowUpdateEncoders) {
            mModules[module].updateEncoderOffset(newOffset);
            return true;
        }
        return false;
    }

    public boolean zeroEncoder(int module) {
        return updateEncoderOffset(module, mModules[module].getEncoderRawPosition());
    }

    public boolean updateAllEncoderOffsets(double[] offsets) {
        if (mAllowUpdateEncoders) {
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].updateEncoderOffset(offsets[i]);
            }
            return true;
        }
        return false;
    }

    public boolean updateEncoderOffsetsFromPersist() {
        if (mAllowUpdateEncoders) {
            boolean allValuesPresent = Preferences.containsKey("drive/offsets/0")
                && Preferences.containsKey("drive/offsets/1")
                && Preferences.containsKey("drive/offsets/2")
                && Preferences.containsKey("drive/offsets/3");
            if (allValuesPresent) {
                double[] offsets = new double[4];
                for (int i = 0; i < mModules.length; i++) {
                    offsets[i] = Preferences.getDouble("drive/offsets/"+i, 0);
                }
                updateAllEncoderOffsets(offsets);
                return true;
            }
            return false;
        }
        return false;
    }

    public boolean saveEncoderOffsetsToPersist() {
        if (mAllowUpdateEncoders) {
            for (int i = 0; i < mModules.length; i++) {
                Preferences.setDouble("drive/offsets/"+i, mModules[i].getEncoderOffset());
            }
            return true;
        }
        return false;
    }

    public void refreshEncoderOffsets() {
        for (int i = 0; i < mModules.length; i++) {
            mEncoderOffsetCache[i] = mModules[i].getEncoderOffset();
        }
    }

    public double getCachedEncoderOffsets(int module) {
        return mEncoderOffsetCache[module];
    }

    public double getLastEncoderPosition(int module) {
        return mLastSwervePositions[module].angle.getRotations();
    }

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != mKinematicLimits) {
            mKinematicLimits = limits;
        }
    }

    public DriveControlState getControlState() {
        return mControlState;
    }

    public String getKinematicLimitsTitle() {
        if (mKinematicLimits == Constants.kUncappedKinematicLimits) {
            return "Uncapped";
        } else if (mKinematicLimits == Constants.kTeleopKinematicLimits) {
            return "Teleop";
        } else if (mKinematicLimits == Constants.kAzimuthOnlyKinematicLimits) {
            return "Azimuth Only";
        } else if (mKinematicLimits == Constants.kFastKinematicLimits) {
            return "Fast";
        } else if (mKinematicLimits == Constants.kSmoothKinematicLimits) {
            return "Smooth";
        } else {
            return "Unknown";
        }
    }

    public ChassisSpeeds getMeasuredSpeeds() {
        return mMeasuredSpeeds;
    }

    private Optional<ChassisSpeeds> updatePathFollower() {
        return Optional.empty();
    }

    private Optional<ChassisSpeeds> updateAutoAlign() {
        if (mControlState != DriveControlState.AUTO_ALIGN && mControlState != DriveControlState.AUTO_ALIGN_Y_THETA) {
            return Optional.empty();
        }

        final double now = Timer.getFPGATimestamp();
        var position = getPose();
        var velocity = Utility.getTwist2dFromChassisSpeeds(getMeasuredSpeeds());

        ChassisSpeeds output = mAutoAlignPlanner.updateAutoAlign(now, position, velocity);

        if (output != null) {
            return Optional.of(output);
        } else {
            return Optional.empty();
        }
    }

    public boolean autoAlignAtTarget() {
        return getPose().relativeTo(getTargetPoint()).getTranslation().getNorm() <= Constants.kLEDClosenessDeadbandMeters 
            && (mControlState == DriveControlState.AUTO_ALIGN || mControlState == DriveControlState.AUTO_ALIGN_Y_THETA);
    }

    public boolean shouldRevertToDeltaIntegration() {
        return !mGyroInputs.connected || mIgnoreGyro;
    }

    public void setIgnoreGyro(boolean ignoreGyro) {
        this.mIgnoreGyro = ignoreGyro;
    }

    public void setDriveAssistFail(boolean failAssist) {
        this.mAllowDriveAssists = !failAssist;
    }
}
