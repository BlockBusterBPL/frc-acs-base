// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveRotations = 0.0;
        public double driveVelocityRotPerSec = 0.0;
        public double driveAppliedCurrent = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelsius = new double[] {};

        public double steerPositionRotations = 0.0;
        public double steerVelocityRotPerSecond = 0.0;
        public double steerAppliedCurrent = 0.0;
        public double[] steerCurrentAmps = new double[] {};
        public double[] steerTempCelsius = new double[] {};
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setDriveSpeedTarget(double speedRotationsPerSecond) {}

    public default void setSteerPositionTarget(double steerAngleRotations) {}

    public default void setDriveBrakeMode(boolean driveBrakeMode) {}

    public default void setSteerBrakeMode(boolean steerBrakeMode) {}

    public default void setDriveKP(double driveKP) {}

    public default void setDriveKI(double driveKI) {}

    public default void setDriveKD(double drivekD) {}

    public default void setDriveKF(double drivekF) {}

    public default void setSteerKP(double steerKP) {}

    public default void setSteerKI(double steerKI) {}

    public default void setSteerKD(double steerKD) {}

    public default void setSteerKF(double steerKF) {}
}
