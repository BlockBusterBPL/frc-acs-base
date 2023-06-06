// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.Alert;
import frc.robot.lib.Alert.AlertType;
import frc.robot.lib.drive.SwerveSetpoint;
import frc.robot.lib.drive.SwerveSetpointGenerator;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private static final double coastThresholdMetersPerSec = 0.05; // Need to be under this to switch to coast when
                                                                   // disabling
    private static final double coastThresholdSecs = 6.0; // Need to be under the above speed for this length of time to
                                                          // switch to coast

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private final Alert alertGyroNotConnected = new Alert(
            "Gyro not connected! Reverting to wheel delta estimation of robot heading, please verify reference pose.",
            AlertType.WARNING);

    private final Alert alertCoastModeEnabled = new Alert(
            "Swerve Drive Coast Mode Enabled! Robot may roll if not on level surface.",
            AlertType.INFO);

    private final Alert alertSteerNeutralMode = new Alert(
            "Swerve Alignment Mode Enabled! Performance may be reduced, as driving in this mode is not intended.",
            AlertType.WARNING);

    private final SwerveModule[] modules;
    private final int kFrontLeftID = 0;
    private final int kFrontRightID = 1;
    private final int kRearLeftID = 2;
    private final int kRearRightID = 3;

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.kWheelPositions);
    SwerveSetpointGenerator generator = new SwerveSetpointGenerator(kinematics);

    private ChassisSpeeds setpoint = new ChassisSpeeds();

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

    private SwerveDrivePoseEstimator odometry;

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
            // TODO: implement a way to change kinematic limits
            var generatedSetpoint = generator.generateSetpoint(Constants.kTeleopKinematicLimits, lastSetpoint,
                    setpoint, Constants.loopPeriodSecs);

            SwerveModuleState[] setpointStates = generatedSetpoint.mModuleStates;

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                optimizedStates[i] = modules[i].setState(setpointStates[i]);
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

        // Update gyro angle
        if (gyroInputs.connected) {
            lastGyroYaw = Rotation2d.fromRotations(gyroInputs.yawAngleRotations);
        } else {
            // estimate rotation angle from wheel deltas when gyro is not connected
            var twist = kinematics.toTwist2d(wheelDeltas);
            lastGyroYaw = lastGyroYaw.plus(new Rotation2d(twist.dtheta).times(Constants.loopPeriodSecs));
        }

        for (int i = 0; i < modules.length; i++) {
            lastSwervePositions[i] = modules[i].getPosition();
        }

        // Update odometry
        lastRobotPose = odometry.update(lastGyroYaw, lastSwervePositions);

        // poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Log 3D odometry pose
        var robotPose3d = new Pose3d(lastRobotPose);
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

        // Run alert checks
        alertGyroNotConnected.set(gyroInputs.connected);
        alertCoastModeEnabled.set(!isBrakeMode);
        alertSteerNeutralMode.set(SwerveModule.getSteerNeutralMode());
    }

    public void swerveDrive(ChassisSpeeds speeds) {
        setpoint = speeds;
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
            modules[i].setState(moduleStates[i]);
        }
    }

    public Pose2d getPose() {
        return lastRobotPose; // TODO
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(gyroInputs.yawAngleRotations); // TODO
    }
}
