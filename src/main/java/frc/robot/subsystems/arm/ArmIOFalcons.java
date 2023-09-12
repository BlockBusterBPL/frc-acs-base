package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.StrictFollower;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ForwardLimitValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;

import frc.robot.Constants;
import frc.robot.lib.phoenixpro.TalonConfigHelper;

public class ArmIOFalcons implements ArmIO {
    ////////// TILT MOTORS \\\\\\\\\\
    private final TalonFX tiltMaster;
    private final TalonFX tiltFollower;

    private final TalonFXConfiguration tiltConfig;

    private final MotionMagicTorqueCurrentFOC tiltMasterControl;
    private final StrictFollower tiltFollowerControl;

    ////////// EXTEND MOTORS \\\\\\\\\\
    private final TalonFX extendMaster;
    private final TalonFX extendFollower;

    private final TalonFXConfiguration extendConfig;

    private final MotionMagicTorqueCurrentFOC extendMasterControl;
    private final StrictFollower extendFollowerControl;

    ////////// WRIST MOTOR \\\\\\\\\\
    private final TalonFX wrist;

    private final TalonFXConfiguration wristConfig;

    private final MotionMagicTorqueCurrentFOC wristControl;

    ///////// STATUS SIGNALS \\\\\\\\\\
    private StatusSignalValue<Double> tiltMasterPosition;
    private StatusSignalValue<Double> tiltMasterVelocity;
    private StatusSignalValue<Double> tiltMasterAppliedCurrent;
    private StatusSignalValue<Double> tiltMasterSuppliedCurrent;
    private StatusSignalValue<Double> tiltMasterTempCelsius;
    private StatusSignalValue<Boolean> tiltMasterReverseSoftLimit;
    private StatusSignalValue<Boolean> tiltMasterForwardSoftLimit;
    private StatusSignalValue<ReverseLimitValue> tiltMasterReverseHardLimit;
    private StatusSignalValue<ForwardLimitValue> tiltMasterForwardHardLimit;

    private StatusSignalValue<Double> tiltFollowerPosition;
    private StatusSignalValue<Double> tiltFollowerVelocity;
    private StatusSignalValue<Double> tiltFollowerAppliedCurrent;
    private StatusSignalValue<Double> tiltFollowerSuppliedCurrent;
    private StatusSignalValue<Double> tiltFollowerTempCelsius;
    private StatusSignalValue<Boolean> tiltFollowerReverseSoftLimit;
    private StatusSignalValue<Boolean> tiltFollowerForwardSoftLimit;
    private StatusSignalValue<ReverseLimitValue> tiltFollowerReverseHardLimit;
    private StatusSignalValue<ForwardLimitValue> tiltFollowerForwardHardLimit;

    private StatusSignalValue<Double> extendMasterPosition;
    private StatusSignalValue<Double> extendMasterVelocity;
    private StatusSignalValue<Double> extendMasterAppliedCurrent;
    private StatusSignalValue<Double> extendMasterSuppliedCurrent;
    private StatusSignalValue<Double> extendMasterTempCelsius;
    private StatusSignalValue<Boolean> extendMasterReverseSoftLimit;
    private StatusSignalValue<Boolean> extendMasterForwardSoftLimit;
    private StatusSignalValue<ReverseLimitValue> extendMasterReverseHardLimit;
    private StatusSignalValue<ForwardLimitValue> extendMasterForwardHardLimit;

    private StatusSignalValue<Double> extendFollowerPosition;
    private StatusSignalValue<Double> extendFollowerVelocity;
    private StatusSignalValue<Double> extendFollowerAppliedCurrent;
    private StatusSignalValue<Double> extendFollowerSuppliedCurrent;
    private StatusSignalValue<Double> extendFollowerTempCelsius;
    private StatusSignalValue<Boolean> extendFollowerReverseSoftLimit;
    private StatusSignalValue<Boolean> extendFollowerForwardSoftLimit;
    private StatusSignalValue<ReverseLimitValue> extendFollowerReverseHardLimit;
    private StatusSignalValue<ForwardLimitValue> extendFollowerForwardHardLimit;

    private StatusSignalValue<Double> wristPosition;
    private StatusSignalValue<Double> wristVelocity;
    private StatusSignalValue<Double> wristAppliedCurrent;
    private StatusSignalValue<Double> wristSuppliedCurrent;
    private StatusSignalValue<Double> wristTempCelsius;
    private StatusSignalValue<Boolean> wristReverseSoftLimit;
    private StatusSignalValue<Boolean> wristForwardSoftLimit;
    private StatusSignalValue<ReverseLimitValue> wristReverseHardLimit;
    private StatusSignalValue<ForwardLimitValue> wristForwardHardLimit;

    private Collection<StatusSignalValue<?>> m_signals = new ArrayList<StatusSignalValue<?>>();

    public ArmIOFalcons() {
        ////////// TILT MOTORS \\\\\\\\\\
        tiltMaster = new TalonFX(30, "canivore");
        tiltFollower = new TalonFX(31, "canivore");
        tiltConfig = TalonConfigHelper.getBaseConfig();

        tiltConfig.Slot0.kP = Constants.ArmSubsystem.Tilt.kP;
        tiltConfig.Slot0.kI = Constants.ArmSubsystem.Tilt.kI;
        tiltConfig.Slot0.kD = Constants.ArmSubsystem.Tilt.kD;
        tiltConfig.Slot0.kV = Constants.ArmSubsystem.Tilt.kV;

        tiltConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Tilt.kMagicVel;
        tiltConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Tilt.kMagicAccel;
        tiltConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Tilt.kMagicJerk;

        tiltMaster.getConfigurator().apply(tiltConfig);
        tiltFollower.getConfigurator().apply(tiltConfig);

        tiltMasterControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false);
        tiltFollowerControl = new StrictFollower(tiltMaster.getDeviceID());

        ////////// EXTEND MOTORS \\\\\\\\\\
        extendMaster = new TalonFX(32, "canivore");
        extendFollower = new TalonFX(33, "canivore");

        extendConfig = TalonConfigHelper.getBaseConfig();

        extendConfig.Slot0.kP = Constants.ArmSubsystem.Extend.kP;
        extendConfig.Slot0.kI = Constants.ArmSubsystem.Extend.kI;
        extendConfig.Slot0.kD = Constants.ArmSubsystem.Extend.kD;
        extendConfig.Slot0.kV = Constants.ArmSubsystem.Extend.kV;

        extendConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Extend.kMagicVel;
        extendConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Extend.kMagicAccel;
        extendConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Extend.kMagicJerk;

        extendMasterControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false);

        extendFollowerControl = new StrictFollower(extendMaster.getDeviceID());

        ////////// WRIST MOTOR \\\\\\\\\\
        wrist = new TalonFX(34, "canivore");

        wristConfig = TalonConfigHelper.getBaseConfig();

        wristConfig.Slot0.kP = Constants.ArmSubsystem.Wrist.kP;
        wristConfig.Slot0.kI = Constants.ArmSubsystem.Wrist.kI;
        wristConfig.Slot0.kD = Constants.ArmSubsystem.Wrist.kD;
        wristConfig.Slot0.kV = Constants.ArmSubsystem.Wrist.kV;

        wristConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Wrist.kMagicVel;
        wristConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Wrist.kMagicAccel;
        wristConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Wrist.kMagicJerk;

        wristControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false);

        ////////// ALL MOTORS \\\\\\\\\\
        refreshFollowers();

        ////////// STATUS SIGNALS \\\\\\\\\\
        tiltMasterPosition = tiltMaster.getPosition();
        tiltMasterVelocity = tiltMaster.getVelocity();
        tiltMasterAppliedCurrent = tiltMaster.getTorqueCurrent();
        tiltMasterSuppliedCurrent = tiltMaster.getSupplyCurrent();
        tiltMasterTempCelsius = tiltMaster.getDeviceTemp();
        tiltMasterReverseSoftLimit = tiltMaster.getFault_ReverseSoftLimit();
        tiltMasterForwardSoftLimit = tiltMaster.getFault_ForwardSoftLimit();
        tiltMasterReverseHardLimit = tiltMaster.getReverseLimit();
        tiltMasterForwardHardLimit = tiltMaster.getForwardLimit();

        tiltFollowerPosition = tiltFollower.getPosition();
        tiltFollowerVelocity = tiltFollower.getVelocity();
        tiltFollowerAppliedCurrent = tiltFollower.getTorqueCurrent();
        tiltFollowerSuppliedCurrent = tiltFollower.getSupplyCurrent();
        tiltFollowerTempCelsius = tiltFollower.getDeviceTemp();
        tiltFollowerReverseSoftLimit = tiltFollower.getFault_ReverseSoftLimit();
        tiltFollowerForwardSoftLimit = tiltFollower.getFault_ForwardSoftLimit();
        tiltFollowerReverseHardLimit = tiltFollower.getReverseLimit();
        tiltFollowerForwardHardLimit = tiltFollower.getForwardLimit();

        extendMasterPosition = extendMaster.getPosition();
        extendMasterVelocity = extendMaster.getVelocity();
        extendMasterAppliedCurrent = extendMaster.getTorqueCurrent();
        extendMasterSuppliedCurrent = extendMaster.getSupplyCurrent();
        extendMasterTempCelsius = extendMaster.getDeviceTemp();
        extendMasterReverseSoftLimit = extendMaster.getFault_ReverseSoftLimit();
        extendMasterForwardSoftLimit = extendMaster.getFault_ForwardSoftLimit();
        extendMasterReverseHardLimit = extendMaster.getReverseLimit();
        extendMasterForwardHardLimit = extendMaster.getForwardLimit();

        extendFollowerPosition = extendFollower.getPosition();
        extendFollowerVelocity = extendFollower.getVelocity();
        extendFollowerAppliedCurrent = extendFollower.getTorqueCurrent();
        extendFollowerSuppliedCurrent = extendFollower.getSupplyCurrent();
        extendFollowerTempCelsius = extendFollower.getDeviceTemp();
        extendFollowerReverseSoftLimit = extendFollower.getFault_ReverseSoftLimit();
        extendFollowerForwardSoftLimit = extendFollower.getFault_ForwardSoftLimit();
        extendFollowerReverseHardLimit = extendFollower.getReverseLimit();
        extendFollowerForwardHardLimit = extendFollower.getForwardLimit();

        wristPosition = wrist.getPosition();
        wristVelocity = wrist.getVelocity();
        wristAppliedCurrent = wrist.getTorqueCurrent();
        wristSuppliedCurrent = wrist.getSupplyCurrent();
        wristTempCelsius = wrist.getDeviceTemp();
        wristReverseSoftLimit = wrist.getFault_ReverseSoftLimit();
        wristForwardSoftLimit = wrist.getFault_ForwardSoftLimit();
        wristReverseHardLimit = wrist.getReverseLimit();
        wristForwardHardLimit = wrist.getForwardLimit();

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
        inputs.tiltForwardHardLimit[1] = tiltFollowerForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;

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
        inputs.extendReverseHardLimit[1] = extendFollowerReverseHardLimit
                .getValue() == ReverseLimitValue.ClosedToGround;
        inputs.extendForwardHardLimit[0] = extendMasterForwardHardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.extendForwardHardLimit[1] = extendFollowerForwardHardLimit
                .getValue() == ForwardLimitValue.ClosedToGround;

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

    @Override
    public void updateOutputs() {
        tiltMaster.setControl(tiltMasterControl);
        extendMaster.setControl(extendMasterControl);
        wrist.setControl(wristControl);
    }

    @Override
    public void refreshFollowers() {
        tiltFollower.setControl(tiltFollowerControl);
        extendFollower.setControl(extendFollowerControl);
    }
}
