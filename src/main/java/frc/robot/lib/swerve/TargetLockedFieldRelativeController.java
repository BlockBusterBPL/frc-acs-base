package frc.robot.lib.swerve;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.control.RadiusController;
import frc.robot.lib.control.SwerveHeadingController;
import frc.robot.lib.geometry.Pose2d;

public class TargetLockedFieldRelativeController implements IDriveController {
    public static TargetLockedFieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    public static TargetLockedFieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new TargetLockedFieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ImprovedChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        if((mSwerveHeadingController
                .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.POLAR_SNAP
                && mSwerveHeadingController.isAtGoal()) || driveInput.changeToMaintainTargetHeading()) {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.POLAR_MAINTAIN);
            mSwerveHeadingController.setGoal(mSwerveHeadingController.calculateAngleToOrigin(mRobotState.getLatestFieldToVehicle().getValue()));
        } else {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.POLAR_SNAP);
            mSwerveHeadingController.setGoal(mSwerveHeadingController.calculateAngleToOrigin(mRobotState.getLatestFieldToVehicle().getValue()));
        }
        return ImprovedChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.kMaxAngularVelocityRadiansPerSecond,
                robotPose.getRotation().toOld()
                );
    }
}
