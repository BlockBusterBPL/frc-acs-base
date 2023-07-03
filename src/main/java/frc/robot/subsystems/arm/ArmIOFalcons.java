package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ForwardLimitValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;

public class ArmIOFalcons implements ArmIO{
    private final TalonFX tiltMaster = new TalonFX(30, "canivore");
    private final TalonFX tiltFollower = new TalonFX(31, "canivore");

    private final TalonFX extendMaster = new TalonFX(32, "canivore");
    private final TalonFX extendFollower = new TalonFX(33, "canivore");
    private final TalonFX wrist = new TalonFX(34, "canivore");

    private StatusSignalValue<Double> tiltMasterPosition = tiltMaster.getPosition();
    private StatusSignalValue<Double> tiltMasterVelocity = tiltMaster.getVelocity();
    private StatusSignalValue<Double> tiltMasterAppliedCurrent = tiltMaster.getTorqueCurrent();
    private StatusSignalValue<Double> tiltMasterSuppliedCurrent = tiltMaster.getSupplyCurrent();
    private StatusSignalValue<Double> tiltMasterTempCelsius = tiltMaster.getDeviceTemp();
    private StatusSignalValue<Boolean> tiltMasterReverseSoftLimit = tiltMaster.getFault_ReverseSoftLimit();
    private StatusSignalValue<Boolean> tiltMasterForwardSoftLimit = tiltMaster.getFault_ForwardSoftLimit();
    private StatusSignalValue<ReverseLimitValue> tiltMasterReverseHardLimit = tiltMaster.getReverseLimit();
    private StatusSignalValue<ForwardLimitValue> tiltMasterForwardHardLimit = tiltMaster.getForwardLimit();

    private StatusSignalValue<Double> tiltFollowerPosition = tiltFollower.getPosition();
    private StatusSignalValue<Double> tiltFollowerVelocity = tiltFollower.getVelocity();
    private StatusSignalValue<Double> tiltFollowerAppliedCurrent = tiltFollower.getTorqueCurrent();
    private StatusSignalValue<Double> tiltFollowerSuppliedCurrent = tiltFollower.getSupplyCurrent();
    private StatusSignalValue<Double> tiltFollowerTempCelsius = tiltFollower.getDeviceTemp();
    private StatusSignalValue<Boolean> tiltFollowerReverseSoftLimit = tiltFollower.getFault_ReverseSoftLimit();
    private StatusSignalValue<Boolean> tiltFollowerForwardSoftLimit = tiltFollower.getFault_ForwardSoftLimit();
    private StatusSignalValue<ReverseLimitValue> tiltFollowerReverseHardLimit = tiltFollower.getReverseLimit();
    private StatusSignalValue<ForwardLimitValue> tiltFollowerForwardHardLimit = tiltFollower.getForwardLimit();

    private StatusSignalValue<Double> extendMasterPosition = extendMaster.getPosition();
    private StatusSignalValue<Double> extendMasterVelocity = extendMaster.getVelocity();
    private StatusSignalValue<Double> extendMasterAppliedCurrent = extendMaster.getTorqueCurrent();
    private StatusSignalValue<Double> extendMasterSuppliedCurrent = extendMaster.getSupplyCurrent();
    private StatusSignalValue<Double> extendMasterTempCelsius = extendMaster.getDeviceTemp();
    private StatusSignalValue<Boolean> extendMasterReverseSoftLimit = extendMaster.getFault_ReverseSoftLimit();
    private StatusSignalValue<Boolean> extendMasterForwardSoftLimit = extendMaster.getFault_ForwardSoftLimit();
    private StatusSignalValue<ReverseLimitValue> extendMasterReverseHardLimit = extendMaster.getReverseLimit();
    private StatusSignalValue<ForwardLimitValue> extendMasterForwardHardLimit = extendMaster.getForwardLimit();

    private StatusSignalValue<Double> extendFollowerPosition = extendFollower.getPosition();
    private StatusSignalValue<Double> extendFollowerVelocity = extendFollower.getVelocity();
    private StatusSignalValue<Double> extendFollowerAppliedCurrent = extendFollower.getTorqueCurrent();
    private StatusSignalValue<Double> extendFollowerSuppliedCurrent = extendFollower.getSupplyCurrent();
    private StatusSignalValue<Double> extendFollowerTempCelsius = extendFollower.getDeviceTemp();
    private StatusSignalValue<Boolean> extendFollowerReverseSoftLimit = extendFollower.getFault_ReverseSoftLimit();
    private StatusSignalValue<Boolean> extendFollowerForwardSoftLimit = extendFollower.getFault_ForwardSoftLimit();
    private StatusSignalValue<ReverseLimitValue> extendFollowerReverseHardLimit = extendFollower.getReverseLimit();
    private StatusSignalValue<ForwardLimitValue> extendFollowerForwardHardLimit = extendFollower.getForwardLimit();

    private StatusSignalValue<Double> wristPosition = wrist.getPosition();
    private StatusSignalValue<Double> wristVelocity = wrist.getVelocity();
    private StatusSignalValue<Double> wristAppliedCurrent = wrist.getTorqueCurrent();
    private StatusSignalValue<Double> wristSuppliedCurrent = wrist.getSupplyCurrent();
    private StatusSignalValue<Double> wristTempCelsius = wrist.getDeviceTemp();
    private StatusSignalValue<Boolean> wristReverseSoftLimit = wrist.getFault_ReverseSoftLimit();
    private StatusSignalValue<Boolean> wristForwardSoftLimit = wrist.getFault_ForwardSoftLimit();
    private StatusSignalValue<ReverseLimitValue> wristReverseHardLimit = wrist.getReverseLimit();
    private StatusSignalValue<ForwardLimitValue> wristForwardHardLimit = wrist.getForwardLimit();

    private Collection<StatusSignalValue<?>> m_signals = new ArrayList<StatusSignalValue<?>>();

    public ArmIOFalcons() {
        m_signals.add(tiltMasterPosition);
        m_signals.add(tiltMasterVelocity);
        m_signals.add(tiltMasterAppliedCurrent);
        m_signals.add(tiltMasterSuppliedCurrent);
        m_signals.add(tiltMasterTempCelsius);
        m_signals.add(tiltMasterReverseSoftLimit);
        m_signals.add(tiltMasterForwardSoftLimit);
        m_signals.add(tiltMasterReverseHardLimit);
        m_signals.add(tiltMasterForwardHardLimit);

        m_signals.add(tiltFollowerPosition);
        m_signals.add(tiltFollowerVelocity);
        m_signals.add(tiltFollowerAppliedCurrent);
        m_signals.add(tiltFollowerSuppliedCurrent);
        m_signals.add(tiltFollowerTempCelsius);
        m_signals.add(tiltFollowerReverseSoftLimit);
        m_signals.add(tiltFollowerForwardSoftLimit);
        m_signals.add(tiltFollowerReverseHardLimit);
        m_signals.add(tiltFollowerForwardHardLimit);

        m_signals.add(extendMasterPosition);
        m_signals.add(extendMasterVelocity);
        m_signals.add(extendMasterAppliedCurrent);
        m_signals.add(extendMasterSuppliedCurrent);
        m_signals.add(extendMasterTempCelsius);
        m_signals.add(extendMasterReverseSoftLimit);
        m_signals.add(extendMasterForwardSoftLimit);
        m_signals.add(extendMasterReverseHardLimit);
        m_signals.add(extendMasterForwardHardLimit);

        m_signals.add(extendFollowerPosition);
        m_signals.add(extendFollowerVelocity);
        m_signals.add(extendFollowerAppliedCurrent);
        m_signals.add(extendFollowerSuppliedCurrent);
        m_signals.add(extendFollowerTempCelsius);
        m_signals.add(extendFollowerReverseSoftLimit);
        m_signals.add(extendFollowerForwardSoftLimit);
        m_signals.add(extendFollowerReverseHardLimit);
        m_signals.add(extendFollowerForwardHardLimit);

        m_signals.add(wristPosition);
        m_signals.add(wristVelocity);
        m_signals.add(wristAppliedCurrent);
        m_signals.add(wristSuppliedCurrent);
        m_signals.add(wristTempCelsius);
        m_signals.add(wristReverseSoftLimit);
        m_signals.add(wristForwardSoftLimit);
        m_signals.add(wristReverseHardLimit);
        m_signals.add(wristForwardHardLimit);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_signals.forEach((s) -> s.refresh());

        inputs.tiltRotations = tiltMasterPosition.getValue();
        inputs.tiltVelocityRotPerSec = tiltMasterVelocity.getValue();
        inputs.tiltAppliedCurrentAmps[0] = tiltMasterAppliedCurrent.getValue();
        inputs.tiltAppliedCurrentAmps[1] = tiltFollowerAppliedCurrent.getValue();
        inputs.tiltSuppliedCurrentAmps[0] = tiltMasterSuppliedCurrent.getValue();
        inputs.tiltSuppliedCurrentAmps[1] = tiltFollowerSuppliedCurrent.getValue();
        inputs.tiltTempCelsius[0] = tiltMasterTempCelsius.getValue();
        inputs.tiltTempCelsius[1] = tiltFollowerTempCelsius.getValue();
        inputs.tiltReverseSoftLimit[0] = tiltMasterReverseSoftLimit.getValue();
        inputs.tiltReverseSoftLimit[1] = tiltFollowerReverseSoftLimit.getValue();
        inputs.tiltForwardSoftLimit[0] = tiltMasterForwardSoftLimit.getValue();
        inputs.tiltForwardSoftLimit[1] = tiltFollowerForwardSoftLimit.getValue();
        inputs.tiltReverseHardLimit[0] = tiltMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.tiltReverseHardLimit[1] = tiltFollowerReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.tiltForwardHardLimit[0] = tiltMasterForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.tiltForwardHardLimit[0] = tiltFollowerForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;

        inputs.extendMeters = extendMasterPosition.getValue();
        inputs.extendVelocityMetersPerSec = extendMasterVelocity.getValue();
        inputs.extendAppliedCurrentAmps[0] = extendMasterAppliedCurrent.getValue();
        inputs.extendAppliedCurrentAmps[1] = extendFollowerAppliedCurrent.getValue();
        inputs.extendSuppliedCurrentAmps[0] = extendMasterSuppliedCurrent.getValue();
        inputs.extendSuppliedCurrentAmps[1] = extendFollowerSuppliedCurrent.getValue();
        inputs.extendTempCelsius[0] = extendMasterTempCelsius.getValue();
        inputs.extendTempCelsius[1] = extendFollowerTempCelsius.getValue();
        inputs.extendReverseSoftLimit[0] = extendMasterReverseSoftLimit.getValue();
        inputs.extendReverseSoftLimit[1] = extendFollowerReverseSoftLimit.getValue();
        inputs.extendForwardSoftLimit[0] = extendMasterForwardSoftLimit.getValue();
        inputs.extendForwardSoftLimit[1] = extendFollowerForwardSoftLimit.getValue();
        inputs.extendReverseHardLimit[0] = extendMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.extendReverseHardLimit[1] = extendFollowerReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.extendForwardHardLimit[0] = extendMasterForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.extendForwardHardLimit[0] = extendFollowerForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;

        inputs.wristRotations = wristPosition.getValue();
        inputs.wristVelocityRotPerSec = wristVelocity.getValue();
        inputs.wristAppliedCurrentAmps = wristAppliedCurrent.getValue();
        inputs.wristSuppliedCurrentAmps = wristSuppliedCurrent.getValue();
        inputs.wristTempCelsius = wristTempCelsius.getValue();
        inputs.wristReverseSoftLimit = wristReverseSoftLimit.getValue();
        inputs.wristForwardSoftLimit = wristForwardSoftLimit.getValue();
        inputs.wristReverseHardLimit = wristReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.wristForwardHardLimit = wristForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }


}
