// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawAngleRotations;
        public double pitchAngleRotations;
        public double rollAngleRotations;

        public boolean connected;
        public boolean calibrating;

        public double quaternionW;
        public double quaternionX;
        public double quaternionY;
        public double quaternionZ;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void calibrate() {}
}
