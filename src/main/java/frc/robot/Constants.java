package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.drive.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.lib.motion.MotionProfileConstraints;

import java.io.IOException;
import java.util.Map;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;

public final class Constants {
    private static final RobotType robot = RobotType.ROBOT_2023_CHASSIS;
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = true;
    
    public static boolean invalidRobotAlertSent = false;
    
    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
                if (!invalidRobotAlertSent) {
                    new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                    .set(true);
                    invalidRobotAlertSent = true;
                }
                return RobotType.ROBOT_2023C;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }
    
    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2023C:
            case ROBOT_2023P:
            case ROBOT_2023_CHASSIS:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            
            case ROBOT_SIMBOT:
            return Mode.SIM;
            
            default:
            return Mode.REAL;
        }
    }
    
    public static final Map<RobotType, String> logFolders =
    Map.of(RobotType.ROBOT_2023C, "/media/sda2/");
    
    public static enum RobotType {
        ROBOT_2023C,
        ROBOT_2023P,
        ROBOT_2023_CHASSIS,
        ROBOT_SIMBOT
    }
    
    public static enum Mode {
        REAL,
        REPLAY,
        SIM
    }
    
    // Function to disable HAL interaction when running without native libs
    public static boolean disableHAL = false;
    
    public static void disableHAL() {
        disableHAL = true;
    }
    
    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final String kRioCANBusName = "rio";
    public static final String kCANivoreCANBusName = "canivore";
    public static final double kCancoderBootAllowanceSeconds = 10.0;
    
    // Drive constants
    /** SDS Mk4 L2 Gear Ratio */
    public static final double kSDS_L2 = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static final double kDriveReduction = kSDS_L2;//(14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveWheelDiameter = Units.inchesToMeters(4.0); /// meters, TODO measure
    public static final double kDriveTrackwidthMeters = 0.61595; // DONE Measure and set trackwidth
    public static final double kDriveWheelbaseMeters = 0.61595; // DONE Measure and set wheelbase
    
    public static final double kMaxVelocityMetersPerSecond = 5.05; //Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;
    
    // Robot constants
    public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot
    public static final double kTrackScrubFactor = 1;
    
    
    /**
    * The maximum angular velocity of the robot in radians per second.
    * <p>
    * This is a measure of how fast the robot can rotate in place.
    */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double kMaxAngularVelocityRadiansPerSecond = 11.386413;
    
    public static final double kScaleTranslationInputs = 0.5;
    public static final double kScaleRotationInputs = 0.2;
    
    public static final KinematicLimits kUncappedKinematicLimits = new KinematicLimits();
    static {
        kUncappedKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }
    
    public static final KinematicLimits kAzimuthOnlyKinematicLimits = new KinematicLimits();
    static {
        kAzimuthOnlyKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kAzimuthOnlyKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kAzimuthOnlyKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }
    
    public static final KinematicLimits kTeleopKinematicLimits = new KinematicLimits();
    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kTeleopKinematicLimits.kMaxDriveAcceleration = kTeleopKinematicLimits.kMaxDriveVelocity / 0.1;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }
    
    public static final KinematicLimits kFastKinematicLimits = new KinematicLimits();
    static {
        kFastKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.2;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }
    
    public static final KinematicLimits kSmoothKinematicLimits = new KinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * .7;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveVelocity / 1.0;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }
    
    public static final Translation2d[] kWheelPositions = {
        // Front left
        new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
        // Front right
        new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
        // Back left
        new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
        // Back right
        new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0)
    };
    
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        kWheelPositions
    );
    
    // // Module Configurations
    // public static final CanDeviceId kBackLeftDriveTalonId = new CanDeviceId(3, kCANivoreCANBusName);
    // public static final CanDeviceId kBackLeftAziTalonId = new CanDeviceId(2, kCANivoreCANBusName);
    // public static final CanDeviceId kBackLeftEncoderPortId = new CanDeviceId(4, kCANivoreCANBusName);
    // public static final Rotation2d kBackLeftAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(272.285) : Rotation2d.fromDegrees(283.62);
    
    // public static final CanDeviceId kBackRightDriveTalonId = new CanDeviceId(6, kCANivoreCANBusName);
    // public static final CanDeviceId kBackRightAziTalonId = new CanDeviceId(5, kCANivoreCANBusName);
    // public static final CanDeviceId kBackRightEncoderPortId = new CanDeviceId(7, kCANivoreCANBusName);
    // public static final Rotation2d kBackRightAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(194.67) : Rotation2d.fromDegrees(201.5);
    
    // public static final CanDeviceId kFrontRightDriveTalonId = new CanDeviceId(12, kCANivoreCANBusName);
    // public static final CanDeviceId kFrontRightAziTalonId = new CanDeviceId(11, kCANivoreCANBusName);
    // public static final CanDeviceId kFrontRightEncoderPortId = new CanDeviceId(13, kCANivoreCANBusName);
    // public static final Rotation2d kFrontRightAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(103.45) : Rotation2d.fromDegrees(67.5);
    
    // public static final CanDeviceId kFrontLeftDriveTalonId = new CanDeviceId(9, kCANivoreCANBusName);
    // public static final CanDeviceId kFrontLeftAziTalonId = new CanDeviceId(8, kCANivoreCANBusName);
    // public static final CanDeviceId kFrontLeftEncoderPortId = new CanDeviceId(10, kCANivoreCANBusName);
    // public static final Rotation2d kFrontLeftAziEncoderOffset = kPracticeBot ? Rotation2d.fromDegrees(14.5) : Rotation2d.fromDegrees(10.75);
    
    // TODO rename these to be steer/drive
    public static final double kMk4AziKp = 0.75;
    public static final double kMk4AziKi = 0;
    public static final double kMk4AziKd = 15;
    
    public static final double kMk4DriveVelocityKp = 0.1;
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.01;
    public static final double kMk4DriveVelocityKf = 1023 / (kMaxVelocityMetersPerSecond / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction / 2048.0 * 10));
    
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final TrapezoidProfile.Constraints kPositionControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocityMetersPerSecond, kMaxVelocityMetersPerSecond);

    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
            0.8 * Constants.kMaxVelocityMetersPerSecond,
            0.8 * -Constants.kMaxVelocityMetersPerSecond,
            0.6 * Constants.kMaxAccelerationMetersPerSecondSquared);
    public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            0.5 * Constants.kMaxAngularSpeedRadiansPerSecond,
            0.5 * -Constants.kMaxAngularSpeedRadiansPerSecond,
            1.0 * Constants.kMaxAngularAccelerationRadiansPerSecondSquared);


    
    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.5; // degree error

    public static final double kSnapSwerveHeadingKp = 0.05;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.0075;

    public static final double kMaintainSwerveHeadingKpHighVelocity = 0.0225;
    public static final double kMaintainSwerveHeadingKiHighVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdHighVelocity = 0.003;

    public static final double kMaintainSwerveHeadingKpLowVelocity = 0.02;  // 0.01;
    public static final double kMaintainSwerveHeadingKiLowVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdLowVelocity = 0.0;

    // Swerve heading controller gains
    public static final double kHeadingControllerKp = 2.54;
    public static final double kHeadingControllerKi = 0.0;
    public static final double kHeadingControllerKd = 0.0;
    public static final double kHeadingControllerKffv = 1.0;
    public static final double kHeadingControllerKffa = 0.0;
    public static final double kHeadingControllerKs = 0.0;

    public static final double kSnapRadiusKp = 2.0;
    public static final double kSnapRadiusKi = 0.0;
    public static final double kSnapRadiusKd = 0.0;

    public static final double kMaintainRadiusKp = 1.5;
    public static final double kMaintainRadiusKi = 0.0;
    public static final double kMaintainRadiusKd = 0.0;
    
    //Pure Pursuit Constants
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; //From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

    public static final class DriveSubsystem {
        public static final Slot0Configs kDrivePIDConfig = new Slot0Configs();
        static {
            kDrivePIDConfig.kP = 5.0;
            kDrivePIDConfig.kI = 0.0;
            kDrivePIDConfig.kD = 0.0;
            kDrivePIDConfig.kV = 2.0;
            kDrivePIDConfig.kS = 0.0;
        }

        public static final Slot0Configs kSteerPIDConfig = new Slot0Configs();
        static {
            kSteerPIDConfig.kP = 60;
            kSteerPIDConfig.kI = 0.0;
            kSteerPIDConfig.kD = 1.0;
            kSteerPIDConfig.kV = 0.0;
            kSteerPIDConfig.kS = 0.0;
        }

        public static final double kSteerKS = 0.0;
        public static final double kSteerKV = 0.0;
        public static final double kSteerKA = 0.0;

        public static final MotionMagicConfigs kSteerMagicConfig = new MotionMagicConfigs();
        static {
            kSteerMagicConfig.MotionMagicCruiseVelocity = 0.25;
            kSteerMagicConfig.MotionMagicAcceleration = 1.0;
            kSteerMagicConfig.MotionMagicJerk = 10;
        }

    }

    public static final class VisionSubsystem {
        public static final String kCameraName = "cam1";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = getAprilTagFieldLayout();
        
        public static AprilTagFieldLayout getAprilTagFieldLayout() {
            try {
                return AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
                return null;
            }
        }
                // AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class ArmSubsystem {
        public static final class Tilt {
            public static final boolean invertMaster = false;
            public static final boolean invertFollower = false;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.0;
            
            public static final double kMagicVel = 0.0;
            public static final double kMagicAccel = 0.0;
            public static final double kMagicJerk = 0.0;

            public static final double kLiberalAllowableError = 0.1;
            public static final double kConservativeAllowableError = 0.05;
            
            public static final double kHomePosition = 0.0;
        }

        public static final class Extend {
            public static final boolean invertMaster = false;
            public static final boolean invertFollower = false;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.0;
            
            public static final double kMagicVel = 0.0;
            public static final double kMagicAccel = 0.0;
            public static final double kMagicJerk = 0.0;

            public static final double kLiberalAllowableError = 0.1;
            public static final double kConservativeAllowableError = 0.02;

            public static final double kHomePosition = 0.0;
        }

        public static final class Wrist {
            public static final boolean invertMaster = false;
            public static final boolean invertFollower = false;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.0;

            public static final double kMagicVel = 0.0;
            public static final double kMagicAccel = 0.0;
            public static final double kMagicJerk = 0.0;

            public static final double kLiberalAllowableError = 0.1;
            public static final double kConservativeAllowableError = 0.05;

            public static final double kHomePosition = 0.0;
        }

        public static final class Gripper {
            public static final boolean kInvert = false;

            public static final double kAutoIntakeConeThrottle = -1.0;
            public static final double kAutoScoreConeThrottle = 1.0;

            public static final double kAutoIntakeCubeThrottle = -1.0;
            public static final double kAutoScoreCubeThrottle = 0.6; 
        }
    }
    
    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robot == RobotType.ROBOT_SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
            System.exit(1);
        }
    }
}