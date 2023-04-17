package frc.robot.lib.swerve;

import frc.robot.Constants;
import frc.robot.lib.control.RadiusController;
import frc.robot.lib.control.SwerveHeadingController;
import frc.robot.lib.geometry.Pose2d;

public class HeadingLockedFieldRelativeController implements IDriveController {
    public static HeadingLockedFieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    public static HeadingLockedFieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new HeadingLockedFieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
        mSwerveHeadingController.setGoal(driveInput.getDesiredCardinalHeading());
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.kMaxAngularVelocityRadiansPerSecond,
                robotPose.getRotation());
    }
}
