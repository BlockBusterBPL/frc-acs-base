// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class PositionTracker {
    private static Translation2d mCurrentRobotPosition;
    private static ChassisSpeeds mCurrentRobotSpeed;


    public static final PositionTracker instance = new PositionTracker();

    public static PositionTracker getInstance() {
        return instance;
    }

    private PositionTracker() {

    }

    public static Translation2d getCurrentRobotPosition() {
        return mCurrentRobotPosition;
    }

    public static ChassisSpeeds getCurrentRobotSpeeds() {
        return mCurrentRobotSpeed;
    }

    public static void setCurrentRobotPosition(Translation2d position) {
        mCurrentRobotPosition = position;
    }

    public static void setCurrentRobotVelocity(ChassisSpeeds speeds) {
        mCurrentRobotSpeed = speeds;
    }
}
