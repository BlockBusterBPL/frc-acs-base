package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.TunablePID;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double tiltRotations = 0.0;
        public double tiltVelocityRotPerSec = 0.0;
        public double tiltAccelRotPerSecSquared = 0.0;
        public double[] tiltAppliedCurrentAmps = {};
        public double[] tiltSuppliedCurrentAmps = {};
        public double[] tiltTempCelsius = {};
        public boolean tiltReverseSoftLimit = false;
        public boolean tiltForwardSoftLimit = false;
        public boolean tiltReverseHardLimit = false;
        public boolean tiltForwardHardLimit = false;

        public double extendMeters = 0.0;
        public double extendVelocityMetersPerSec = 0.0;
        public double extendAccelMetersPerSecSquared = 0.0;
        public double[] extendAppliedCurrentAmps = {};
        public double[] extendSuppliedCurrentAmps = {};
        public double[] extendTempCelsius = {};
        public boolean extendReverseSoftLimit = false;
        public boolean extendForwardSoftLimit = false;
        public boolean extendReverseHardLimit = false;
        public boolean extendForwardHardLimit = false;

        public double wristRotations = 0.0;
        public double wristVelocityRotPerSec = 0.0;
        public double wristAccelRotPerSecSquared = 0.0;
        public double[] wristAppliedCurrentAmps = {};
        public double[] wristSuppliedCurrentAmps = {};
        public double[] wristTempCelsius = {};
        public boolean wristReverseSoftLimit = false;
        public boolean wristForwardSoftLimit = false;
        public boolean wristReverseHardLimit = false;
        public boolean wristForwardHardLimit = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setTiltTarget(double rotations) {}

    public default void setExtendTarget(double meters) {}

    public default void setWristTarget(double rotations) {}

    public default void setTiltFeedForward(double amps) {}

    public default void setExtendFeedForward(double amps) {}

    public default void setWristFeedForward(double amps) {}

    public default Optional<TunablePID> getTiltPID() {return Optional.empty();}

    public default Optional<TunablePID> getExtendPID() {return Optional.empty();}

    public default Optional<TunablePID> getWristPID() {return Optional.empty();}

    public default void tiltThrottle(double throttle) {}

    public default void extendThrottle(double throttle) {}

    public default void wristThrottle(double throttle) {}
}
