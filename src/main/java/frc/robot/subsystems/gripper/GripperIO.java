package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    @AutoLog
    public static class GripperIOInputs {
        public double motorSpeedRotationsPerSecond = 0.0;
        public boolean cubeInIntake = false;
        public boolean coneInIntake = false;
    }

    public default void updateInputs(GripperIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void set(double throttle) {}
}
