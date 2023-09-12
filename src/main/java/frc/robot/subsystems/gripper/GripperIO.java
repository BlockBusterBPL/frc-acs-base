package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    @AutoLog
    public static class GripperIOInputs {
        public double[] motorSpeedRotationsPerSecond = {0.0, 0.0, 0.0};
        public double[] motorCurrentAmps = {0.0, 0.0, 0.0};
        public double[] motorTempCelsius = {0.0, 0.0, 0.0};
        public boolean cubeInIntake = false;
        public boolean coneInIntake = false;
    }

    public default void updateInputs(GripperIOInputs inputs) {}

    public default void updateOutputs() {}
    
    /**
     * Set the throttle of a single motor intake.
     * @param throttle
     */
    public default void setSingleMotor(double throttle) {}

    /**
     * Set the throttle of a cube intake on an independent intake.
     * @param throttle
     */
    public default void setCube(double throttle) {}
    
    /**
     * Set the throttle of a cone intake on an independent intake.
     * @param throttle
     */
    public default void setCone(double throttle) {}
}
