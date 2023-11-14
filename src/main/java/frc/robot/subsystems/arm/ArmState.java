package frc.robot.subsystems.arm;

import frc.robot.lib.util.Util;

import static frc.robot.Constants.ArmSubsystem.*;

public class ArmState {
    public double tilt = Tilt.kHomePosition;
    public double extend = Extend.kHomePosition;
    public double wrist = Wrist.kHomePosition;
    public double tiltTolerance = Tilt.kLiberalAllowableError;
    public double extendTolerance = Extend.kLiberalAllowableError;
    public double wristTolerance = Wrist.kLiberalAllowableError;
    public Action action = Action.NEUTRAL;
    public ArmSend send = ArmSend.MEDIUM;

    /**  */
    public enum Action {
        /** stop intake rollers */
        NEUTRAL,
        /** Run gripper out */
        SCORING,
        /** Run gripper in */
        INTAKING
    }

    public enum ArmSend {
        LOW,
        MEDIUM,
        FULL
    }

    public ArmState(double tilt, double extend, double wrist, double tiltTolerance, double extendTolerance, double wristTolerance, Action action, ArmSend send) {
        this.tilt = tilt;
        this.extend = extend;
        this.wrist = wrist;
        this.tiltTolerance = tiltTolerance;
        this.extendTolerance = extendTolerance;
        this.wristTolerance = wristTolerance;
    }

    public ArmState(ArmState other) {
        this(other.tilt, other.extend, other.wrist, other.tiltTolerance, other.extendTolerance, other.wristTolerance, other.action, other.send);
    }

    public ArmState(double tilt, double extend, double wrist) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, Action.NEUTRAL, ArmSend.MEDIUM);
    }

    public ArmState(double tilt, double extend, double wrist, Action action) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, action, ArmSend.MEDIUM);
    }

    public ArmState(double tilt, double extend, double wrist, Action action, ArmSend send) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, action, send);
    }

    public ArmState() {}

    public static ArmState generateWithFailsafeParameters(double tilt, double extend, double wrist) {
        return new ArmState(tilt, extend, wrist, 
            Tilt.kConservativeAllowableError, 
            Extend.kConservativeAllowableError, 
            Wrist.kConservativeAllowableError, 
            Action.NEUTRAL, ArmSend.LOW);
    }

    public double getTilt() {
        return Util.limit(tilt, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double getExtend() {
        return Util.limit(extend, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double getWrist() {
        return Util.limit(wrist, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public boolean isInRange(ArmState other) {
        return isInRange(other, Math.min(this.tiltTolerance, other.tiltTolerance), Math.min(this.extendTolerance, other.extendTolerance), Math.min(this.wristTolerance, other.wristTolerance));
    }

    public boolean isInRange(ArmState other, double tiltAllowableError, double extendAllowableError, double wristAllowableError) {
        return Util.epsilonEquals(this.getTilt(), other.getTilt(), tiltAllowableError)
                && Util.epsilonEquals(this.getExtend(), other.getExtend(), extendAllowableError)
                && Util.epsilonEquals(this.getWrist(), other.getWrist(), wristAllowableError);
    }

    public String toString() {
        return "Arm state: (" + getTilt() + ", " + getExtend() + ", " + getWrist() + ", " + action.toString() + ")";
    }
}
