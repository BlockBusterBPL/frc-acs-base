package frc.robot.lib.swerve;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.control.RadiusController;
import frc.robot.lib.control.SwerveHeadingController;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.swerve.IDriveController;

public class FieldRelativeController implements IDriveController {
    public static FieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    public static FieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new FieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        if ((mSwerveHeadingController
                .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                && mSwerveHeadingController.isAtGoal()) || driveInput.changeHeadingSetpoint()) {
            mSwerveHeadingController
                    .setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
            mSwerveHeadingController.setGoal(mRobotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees());
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                    driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                    mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.kMaxAngularVelocityRadiansPerSecond,
                    robotPose.getRotation());
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getRotation() * Constants.kMaxAngularVelocityRadiansPerSecond * Constants.kScaleRotationInputs,
                robotPose.getRotation());
    }
}
