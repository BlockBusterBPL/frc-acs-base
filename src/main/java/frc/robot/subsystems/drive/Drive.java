// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
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
import frc.robot.lib.drive.SwerveSetpoint;
import frc.robot.lib.drive.SwerveSetpointGenerator;
import frc.robot.lib.drive.SwerveSetpointGenerator.KinematicLimits;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private static final double coastThresholdMetersPerSec = 0.05; // Need to be under this to switch to coast when
                                                                   // disabling
    private static final double coastThresholdSecs = 6.0; // Need to be under the above speed for this length of time to
                                                          // switch to coast

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private final SwerveModule[] modules;
    private final int kFrontLeftID = 0;
    private final int kFrontRightID = 1;
    private final int kRearLeftID = 2;
    private final int kRearRightID = 3;

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.kWheelPositions);
    SwerveSetpointGenerator generator = new SwerveSetpointGenerator(kinematics);

    
    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private SwerveSetpoint lastSetpoint = new SwerveSetpoint(setpoint, lastSetpointStates);

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
    private Twist2d fieldVelocity = new Twist2d();
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
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        for (SwerveModule swerveModule : modules) {
            swerveModule.periodic();
        }

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
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
        } else {

            var setpointTwist = new Pose2d().log(
                    new Pose2d(
                            setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                            setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                            Rotation2d.fromRotations(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)));

            var adjustedSpeeds = new ChassisSpeeds(
                    setpointTwist.dx / Constants.loopPeriodSecs,
                    setpointTwist.dy / Constants.loopPeriodSecs,
                    setpointTwist.dtheta / Constants.loopPeriodSecs);

            var generatedSetpoint = generator.generateSetpoint(Constants.kTeleopKinematicLimits, lastSetpoint, adjustedSpeeds, Constants.loopPeriodSecs);
            
            SwerveModuleState[] setpointStates = generatedSetpoint.mModuleStates;

            // Set to previous angles if velocity is zero
            // if (adjustedSpeeds.vxMetersPerSecond == 0.0
            //         && adjustedSpeeds.vyMetersPerSecond == 0.0
            //         && adjustedSpeeds.omegaRadiansPerSecond == 0) {
            //     for (int i = 0; i < 4; i++) {
            //         setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
            //     }
            // }
            lastSetpointStates = setpointStates;

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

        // Update odometry
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getDistance() - lastModulePositionsMeters[i]),
                    modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getDistance();
        }
        // var twist = kinematics.toTwist2d(wheelDeltas);
        // var gyroYaw = new Rotation2d(gyroInputs.yawAngleRotations);
        // if (gyroInputs.connected) {
        //     twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        // }
        // lastGyroYaw = gyroYaw;

        for (int i = 0; i < modules.length; i++) {
            lastSwervePositions[i] = modules[i].getPosition();
        }

        var inverseSpeeds = kinematics.toChassisSpeeds(measuredStates);
        var yawDelta = new Rotation2d(inverseSpeeds.omegaRadiansPerSecond).times(Constants.loopPeriodSecs);
        lastGyroYaw = lastGyroYaw.plus(yawDelta);
        lastRobotPose = odometry.update(lastGyroYaw, lastSwervePositions);

        // poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Log 3D odometry pose
        Pose3d robotPose3d = new Pose3d(getPose());
        // robotPose3d =
        // robotPose3d
        // .exp(
        // new Twist3d(
        // 0.0,
        // 0.0,
        // Math.abs(gyroInputs.pitchAngleRotations) * trackWidthX.get() / 2.0,
        // 0.0,
        // gyroInputs.pitchPositionRad,
        // 0.0))
        // .exp(
        // new Twist3d(
        // 0.0,
        // 0.0,
        // Math.abs(gyroInputs.rollPositionRad) * trackWidthY.get() / 2.0,
        // gyroInputs.rollPositionRad,
        // 0.0,
        // 0.0));
        Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

        // Update field velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getRotation());
        fieldVelocity = new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                        ? gyroInputs.yawVelocityRotationsPerSecond
                        : chassisSpeeds.omegaRadiansPerSecond);

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
