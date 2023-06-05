package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.TunablePID;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double tiltRotations;
        public double tiltVelocityRotPerSec;
        public double tiltAccelRotPerSecSquared;
        public double[] tiltAppliedCurrentAmps;
        public double[] tiltSuppliedCurrentAmps;
        public double[] tiltTempCelsius;
        public boolean tiltReverseSoftLimit;
        public boolean tiltForwardSoftLimit;
        public boolean tiltReverseHardLimit;
        public boolean tiltForwardHardLimit;

        public double extendMeters;
        public double extendVelocityMetersPerSec;
        public double extendAccelMetersPerSecSquared;
        public double[] extendAppliedCurrentAmps;
        public double[] extendSuppliedCurrentAmps;
        public double[] extendTempCelsius;
        public boolean extendReverseSoftLimit;
        public boolean extendForwardSoftLimit;
        public boolean extendReverseHardLimit;
        public boolean extendForwardHardLimit;

        public double wristRotations;
        public double wristVelocityRotPerSec;
        public double wristAccelRotPerSecSquared;
        public double[] wristAppliedCurrentAmps;
        public double[] wristSuppliedCurrentAmps;
        public double[] wristTempCelsius;
        public boolean wristReverseSoftLimit;
        public boolean wristForwardSoftLimit;
        public boolean wristReverseHardLimit;
        public boolean wristForwardHardLimit;
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
