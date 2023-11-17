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
    private static final double coastThresholdMetersPerSec = 0.05; // Need to be under this to switch to coast when
                                                                   // disabling
    private static final double coastThresholdSecs = 6.0; // Need to be under the above speed for this length of time to
                                                          // switch to coast

    public enum DriveControlState {
        OPEN_LOOP("OL Velocity"),
        VELOCITY_CONTROL("CL Velocity"),
        PATH_FOLLOWING("CL Pathfinding"),
        AUTO_ALIGN("CL Auto Align"),
        AUTO_ALIGN_Y_THETA("CL Auto Align 2"),
        X_MODE("OL X Mode");

        public String title;

        DriveControlState(String title) {
            this.title = title;
        }
    }

    private DriveControlState mControlState = DriveControlState.VELOCITY_CONTROL;
    private boolean allowDriveAssists = true;
    private AutoAlignMotionPlanner mAutoAlignPlanner = new AutoAlignMotionPlanner();
    private Pose2d mTargetPoint = new Pose2d();
    private KinematicLimits mKinematicLimits = Constants.kUncappedKinematicLimits;

    public static final SwerveModuleState[] X_MODE_STATES = {
        new SwerveModuleState(0, Rotation2d.fromRotations(-0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(0.125)),
        new SwerveModuleState(0, Rotation2d.fromRotations(-0.125))
    };

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;
    private boolean ignoreGyro = false;

    private final Alert alertGyroNotConnected = new Alert(
            "Gyro not connected! " + (Constants.getRobot() == RobotType.ROBOT_SIMBOT ? "This warning is expected when using the SimBot." : ""),
            AlertType.ERROR);

    private final Alert alertGyroManualFail = new Alert(
            "Gyro inputs manually ignored by driver.", 
            AlertType.WARNING);

    private final Alert alertUsingDeltaIntegration = new Alert(
            "Pose angle reverted to wheel delta integration mode, please monitor robot pose. Using autonomous is NOT RECCOMENDED!",
            AlertType.ERROR);

    private final Alert alertCoastModeEnabled = new Alert(
            "Swerve Drive Coast Mode Enabled.",
            AlertType.INFO);

    private final Alert alertSteerNeutralMode = new Alert(
            "Swerve Steering Neutral Mode Enabled! Performance may be reduced, as driving in this mode is not intended.",
            AlertType.WARNING);
    
    private final Alert alertOffsetsNotSafe = new Alert(
            "Swerve offset safety not enabled! Module offsets could be updated by accident", 
            AlertType.WARNING);

    private final SwerveModule[] modules;
    private final int kFrontLeftID = 0;
    private final int kFrontRightID = 1;
    private final int kRearLeftID = 2;
    private final int kRearRightID = 3;

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.kWheelPositions);
    SwerveSetpointGenerator generator = new SwerveSetpointGenerator(kinematics);

    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

    private SwerveSetpoint lastSetpoint = SwerveSetpoint.FOUR_WHEEL_IDENTITY;

    private boolean isBrakeMode = false;
    private Timer lastMovementTimer = new Timer();

    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private SwerveModulePosition[] lastSwervePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Pose2d lastRobotPose = new Pose2d();
    private Rotation3d robotRotation3d = new Rotation3d();

    private SwerveDrivePoseEstimator odometry;

    private Rotation2d autonTarget;

    private boolean allowUpdateEncoders = false;

    public SendableChooser<Boolean> encoderUpdateChooser = new SendableChooser<>();
    private double[] encoderOffsetCache;

    public Drive(GyroIO gyroIO, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO, SwerveModuleIO rearLeftIO,
            SwerveModuleIO rearRightIO) {
        this.gyroIO = gyroIO;
        this.gyroInputs = new GyroIOInputsAutoLogged();
        modules = new SwerveModule[4];

        modules[kFrontLeftID] = new SwerveModule(frontLeftIO, kFrontLeftID);
        modules[kFrontRightID] = new SwerveModule(frontRightIO, kFrontRightID);
        modules[kRearLeftID] = new SwerveModule(rearLeftIO, kRearLeftID);
        modules[kRearRightID] = new SwerveModule(rearRightIO, kRearRightID);

        odometry = new SwerveDrivePoseEstimator(kinematics, lastGyroYaw, lastSwervePositions, getPose());

        autonTarget = new Rotation2d();

        encoderUpdateChooser.setDefaultOption("Disabled (Safe)", false);
        encoderUpdateChooser.addOption("ENABLED (UNSAFE)", true);
        encoderOffsetCache = new double[4];

        lastMovementTimer.reset();
    }

    @Override
    public void periodic() {
        // Update subsystem inputs - this should always be the first thing in periodic()
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        for (SwerveModule swerveModule : modules) {
            swerveModule.periodic();
        }

        // Update battery simulation in sim mode
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            for (SwerveModule module : modules) {
                simCurrent += module.getTotalCurrent();
            }
            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
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
                    break;
                case OPEN_LOOP:
                case VELOCITY_CONTROL:
                    setKinematicLimits(Constants.kTeleopKinematicLimits);
                    break;
                case AUTO_ALIGN:
                case AUTO_ALIGN_Y_THETA:
                    setKinematicLimits(Constants.kUncappedKinematicLimits);
                    driveSetpointOverride = updateAutoAlign();
                    break;
                case X_MODE:
                    setKinematicLimits(Constants.kUncappedKinematicLimits);
                default:
                    break;
            }

            if (driveSetpointOverride.isPresent() && allowDriveAssists) {
                setSetpoint(driveSetpointOverride.get());
            }

            if (mControlState == DriveControlState.X_MODE) {
                SwerveSetpoint generatedSetpoint = generator.generateSetpointForXMode(mKinematicLimits, lastSetpoint, Constants.loopPeriodSecs);
                lastSetpoint = generatedSetpoint;
                setpointStates = generatedSetpoint.mModuleStates;
            } else {
                SwerveSetpoint generatedSetpoint = generator.generateSetpoint(mKinematicLimits, lastSetpoint,
                    setpoint, Constants.loopPeriodSecs);
                lastSetpoint = generatedSetpoint;
                setpointStates = generatedSetpoint.mModuleStates;
            }
            

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                optimizedStates[i] = modules[i].setStateClosedLoop(setpointStates[i]);
            }

            // Log setpoint states
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getDistance() - lastModulePositionsMeters[i]),
                    modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getDistance();
        }

        // Calculate robot velocity from measured chassis speeds
        var inverseSpeeds = kinematics.toChassisSpeeds(measuredStates);
        measuredSpeeds = inverseSpeeds;

        // Update gyro angle
        if (!shouldRevertToDeltaIntegration()) {
            lastGyroYaw = Rotation2d.fromRotations(gyroInputs.yawAngleRotations);
        } else {
            // estimate rotation angle from wheel deltas when gyro is not connected
            lastGyroYaw = lastGyroYaw
                    .plus(new Rotation2d(inverseSpeeds.omegaRadiansPerSecond).times(Constants.loopPeriodSecs));
        }

        for (int i = 0; i < modules.length; i++) {
            lastSwervePositions[i] = modules[i].getPosition();
        }

        // Update odometry
        lastRobotPose = odometry.update(lastGyroYaw, lastSwervePositions);

        // poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Log 3D odometry pose
        var robotTranslation3d = new Translation3d(
                lastRobotPose.getX(),
                lastRobotPose.getY(),
                0);

        robotRotation3d = new Rotation3d(
                gyroInputs.pitchAngleRotations * 2 * Math.PI,
                gyroInputs.rollAngleRotations * 2 * Math.PI,
                lastGyroYaw.getRadians());

        var robotPose3d = new Pose3d(robotTranslation3d, robotRotation3d);

        Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

        // Update brake mode
        boolean stillMoving = false;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(modules[i].getVelocity()) > coastThresholdMetersPerSec) {
                stillMoving = true;
            }
        }
        if (stillMoving)
            lastMovementTimer.reset();
        if (DriverStation.isEnabled()) {
            if (!isBrakeMode) {
                isBrakeMode = true;
                for (var module : modules) {
                    module.setDriveBrakeMode(true);
                }
            }
        } else {
            if (isBrakeMode && lastMovementTimer.hasElapsed(coastThresholdSecs)) {
                isBrakeMode = false;
                for (var module : modules) {
                    module.setDriveBrakeMode(false);
                }
            }
        }

        allowUpdateEncoders = encoderUpdateChooser.getSelected();

        // Run alert checks
        alertGyroNotConnected.set(!gyroInputs.connected);
        alertGyroManualFail.set(ignoreGyro);
        alertUsingDeltaIntegration.set(shouldRevertToDeltaIntegration());
        alertCoastModeEnabled.set(!isBrakeMode);
        alertSteerNeutralMode.set(modules[0].getSteerNeutralMode());
        alertOffsetsNotSafe.set(allowUpdateEncoders);
    }

    public void setVelocityClosedLoop(ChassisSpeeds speeds) {
        mControlState = DriveControlState.VELOCITY_CONTROL;
        setpoint = speeds;
    }

    public void setVelocityOpenLoop(ChassisSpeeds speeds) {
        mControlState = DriveControlState.OPEN_LOOP;
        setpoint = speeds;
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

    public void setSetpoint(ChassisSpeeds speeds) {
        setpoint = speeds;
    }

    public Pose2d getTargetPoint() {
        return mTargetPoint;
    }

    public void reseedRotation() {
        odometry.resetPosition(lastGyroYaw, lastSwervePositions, new Pose2d(lastRobotPose.getTranslation(), new Rotation2d()));
    }
    
    public void stop() {
        setVelocityClosedLoop(new ChassisSpeeds());
    }

    public void setXMode() {
        mControlState = DriveControlState.X_MODE;
    }

    public Pose2d getPose() {
        return lastRobotPose;
    }

    public Rotation3d getGyroAngle() {
        return robotRotation3d;
    }

    public Rotation3d getFieldOrientation() {
        return robotRotation3d.minus(new Rotation3d(0, 0, getGyroAngle().getZ()));
    }

    public Rotation2d getAutonTarget() {
        return autonTarget;
    }

    public void setAutonTarget(Rotation2d autonTarget) {
        this.autonTarget = autonTarget;
    }

    public void addVisionPose(VisionPose pose) {
        odometry.addVisionMeasurement(pose.pose.toPose2d(), pose.timestampSeconds, pose.stddevs);
    }

    public boolean updateEncoderOffset(int module, double newOffset) {
        if (allowUpdateEncoders) {
            modules[module].updateEncoderOffset(newOffset);
            return true;
        }
        return false;
    }

    public boolean zeroEncoder(int module) {
        return updateEncoderOffset(module, modules[module].getEncoderRawPosition());
    }

    public boolean updateAllEncoderOffsets(double[] offsets) {
        if (allowUpdateEncoders) {
            for (int i = 0; i < modules.length; i++) {
                modules[i].updateEncoderOffset(offsets[i]);
            }
            return true;
        }
        return false;
    }

    public boolean updateEncoderOffsetsFromPersist() {
        if (allowUpdateEncoders) {
            boolean allValuesPresent = Preferences.containsKey("drive/offsets/0")
                && Preferences.containsKey("drive/offsets/1")
                && Preferences.containsKey("drive/offsets/2")
                && Preferences.containsKey("drive/offsets/3");
            if (allValuesPresent) {
                double[] offsets = new double[4];
                for (int i = 0; i < modules.length; i++) {
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
        if (allowUpdateEncoders) {
            for (int i = 0; i < modules.length; i++) {
                Preferences.setDouble("drive/offsets/"+i, modules[i].getEncoderOffset());
            }
            return true;
        }
        return false;
    }

    public void refreshEncoderOffsets() {
        for (int i = 0; i < modules.length; i++) {
            encoderOffsetCache[i] = modules[i].getEncoderOffset();
        }
    }

    public double getCachedEncoderOffsets(int module) {
        return encoderOffsetCache[module];
    }

    public double getLastEncoderPosition(int module) {
        return lastSwervePositions[module].angle.getRotations();
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
        return measuredSpeeds;
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
            return Optional.of(ChassisSpeeds.fromFieldRelativeSpeeds(output, position.getRotation()));
        } else {
            return Optional.empty();
        }
    }

    public boolean shouldRevertToDeltaIntegration() {
        return !gyroInputs.connected || ignoreGyro;
    }

    public void setIgnoreGyro(boolean ignoreGyro) {
        this.ignoreGyro = ignoreGyro;
    }

    public void setDriveAssistFail(boolean failAssist) {
        this.allowDriveAssists = !failAssist;
    }
}
