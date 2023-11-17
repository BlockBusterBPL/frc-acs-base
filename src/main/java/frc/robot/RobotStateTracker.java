// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class RobotStateTracker {
    private Translation2d mCurrentRobotPosition;
    private ChassisSpeeds mCurrentRobotSpeed;
    private Pose2d mCurrentRobotPose;

    private boolean mAutoAlignActive;
    private boolean mAutoAlignComplete;

    public static final RobotStateTracker instance = new RobotStateTracker();

    public static RobotStateTracker getInstance() {
        return instance;
    }

    private RobotStateTracker() {

    }

    public Translation2d getCurrentRobotPosition() {
        return mCurrentRobotPosition;
    }

    public ChassisSpeeds getCurrentRobotSpeeds() {
        return mCurrentRobotSpeed;
    }

    public Pose2d getCurrentRobotPose() {
        return mCurrentRobotPose;
    }

    public boolean getAutoAlignActive() {
        return mAutoAlignActive;
    }

    public boolean getAutoAlignComplete() {
        return mAutoAlignComplete;
    }

    public void setCurrentRobotPosition(Translation2d position) {
        mCurrentRobotPosition = position;
    }

    public void setCurrentRobotVelocity(ChassisSpeeds speeds) {
        mCurrentRobotSpeed = speeds;
    }
}