package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.TunablePID;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double tiltRotations = 0.0;
        public double tiltVelocityRotPerSec = 0.0;
        public double[] tiltAppliedCurrentAmps = {0.0, 0.0};
        public double[] tiltSuppliedCurrentAmps = {0.0, 0.0};
        public double[] tiltTempCelsius = {0.0, 0.0};
        public boolean[] tiltReverseSoftLimit = {false, false};
        public boolean[] tiltForwardSoftLimit = {false, false};
        public boolean[] tiltReverseHardLimit = {false, false};
        public boolean[] tiltForwardHardLimit = {false, false};

        public double extendMeters = 0.0;
        public double extendVelocityMetersPerSec = 0.0;
        public double[] extendAppliedCurrentAmps = {0.0, 0.0};
        public double[] extendSuppliedCurrentAmps = {0.0, 0.0};
        public double[] extendTempCelsius = {0.0, 0.0};
        public boolean[] extendReverseSoftLimit = {false, false};
        public boolean[] extendForwardSoftLimit = {false, false};
        public boolean[] extendReverseHardLimit = {false, false};
        public boolean[] extendForwardHardLimit = {false, false};

        public double wristRotations = 0.0;
        public double wristVelocityRotPerSec = 0.0;
        public double wristAppliedCurrentAmps = 0.0;
        public double wristSuppliedCurrentAmps = 0.0;
        public double wristTempCelsius = 0.0;
        public boolean wristReverseSoftLimit = false;
        public boolean wristForwardSoftLimit = false;
        public boolean wristReverseHardLimit = false;
        public boolean wristForwardHardLimit = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void refreshFollowers() {}

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
