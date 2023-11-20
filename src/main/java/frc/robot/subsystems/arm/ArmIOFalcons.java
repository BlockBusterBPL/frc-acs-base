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
    private StatusSignalValue<Double> tiltMasterSuppliedCurrent;
    private StatusSignalValue<Double> tiltMasterTempCelsius;
    private StatusSignalValue<ReverseLimitValue> tiltMasterReverseHardLimit;
    private StatusSignalValue<Boolean> tiltMasterForwardSoftLimit;

    private StatusSignalValue<Double> tiltFollowerPosition;
    private StatusSignalValue<Double> tiltFollowerVelocity;
    private StatusSignalValue<Double> tiltFollowerSuppliedCurrent;
    private StatusSignalValue<Double> tiltFollowerTempCelsius;
    private StatusSignalValue<ReverseLimitValue> tiltFollowerReverseHardLimit;
    private StatusSignalValue<Boolean> tiltFollowerForwardSoftLimit;

    private StatusSignalValue<Double> extendMasterPosition;
    private StatusSignalValue<Double> extendMasterVelocity;
    private StatusSignalValue<Double> extendMasterSuppliedCurrent;
    private StatusSignalValue<Double> extendMasterTempCelsius;
    private StatusSignalValue<ReverseLimitValue> extendMasterReverseHardLimit;
    private StatusSignalValue<Boolean> extendMasterForwardSoftLimit;

    private StatusSignalValue<Double> extendFollowerPosition;
    private StatusSignalValue<Double> extendFollowerVelocity;
    private StatusSignalValue<Double> extendFollowerSuppliedCurrent;
    private StatusSignalValue<Double> extendFollowerTempCelsius;
    private StatusSignalValue<ReverseLimitValue> extendFollowerReverseHardLimit;
    private StatusSignalValue<Boolean> extendFollowerForwardSoftLimit;

    private StatusSignalValue<Double> wristPosition;
    private StatusSignalValue<Double> wristVelocity;
    private StatusSignalValue<Double> wristSuppliedCurrent;
    private StatusSignalValue<Double> wristTempCelsius;
    private StatusSignalValue<ReverseLimitValue> wristReverseHardLimit;
    private StatusSignalValue<Boolean> wristForwardSoftLimit;

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
        configMotors();
        refreshFollowers();

        ////////// STATUS SIGNALS \\\\\\\\\\
        tiltMasterPosition = tiltMaster.getPosition();
        tiltMasterVelocity = tiltMaster.getVelocity();
        tiltMasterSuppliedCurrent = tiltMaster.getSupplyCurrent();
        tiltMasterTempCelsius = tiltMaster.getDeviceTemp();
        tiltMasterForwardSoftLimit = tiltMaster.getFault_ForwardSoftLimit();
        tiltMasterReverseHardLimit = tiltMaster.getReverseLimit();

        tiltFollowerPosition = tiltFollower.getPosition();
        tiltFollowerVelocity = tiltFollower.getVelocity();
        tiltFollowerSuppliedCurrent = tiltFollower.getSupplyCurrent();
        tiltFollowerTempCelsius = tiltFollower.getDeviceTemp();
        tiltFollowerForwardSoftLimit = tiltFollower.getFault_ForwardSoftLimit();
        tiltFollowerReverseHardLimit = tiltFollower.getReverseLimit();

        extendMasterPosition = extendMaster.getPosition();
        extendMasterVelocity = extendMaster.getVelocity();
        extendMasterSuppliedCurrent = extendMaster.getSupplyCurrent();
        extendMasterTempCelsius = extendMaster.getDeviceTemp();
        extendMasterForwardSoftLimit = extendMaster.getFault_ForwardSoftLimit();
        extendMasterReverseHardLimit = extendMaster.getReverseLimit();

        extendFollowerPosition = extendFollower.getPosition();
        extendFollowerVelocity = extendFollower.getVelocity();
        extendFollowerSuppliedCurrent = extendFollower.getSupplyCurrent();
        extendFollowerTempCelsius = extendFollower.getDeviceTemp();
        extendFollowerForwardSoftLimit = extendFollower.getFault_ForwardSoftLimit();
        extendFollowerReverseHardLimit = extendFollower.getReverseLimit();

        wristPosition = wrist.getPosition();
        wristVelocity = wrist.getVelocity();
        wristSuppliedCurrent = wrist.getSupplyCurrent();
        wristTempCelsius = wrist.getDeviceTemp();
        wristForwardSoftLimit = wrist.getFault_ForwardSoftLimit();
        wristReverseHardLimit = wrist.getReverseLimit();

        m_signals.add(tiltMasterPosition);
        m_signals.add(tiltMasterVelocity);
        m_signals.add(tiltMasterSuppliedCurrent);
        m_signals.add(tiltMasterTempCelsius);
        m_signals.add(tiltMasterForwardSoftLimit);
        m_signals.add(tiltMasterReverseHardLimit);

        m_signals.add(tiltFollowerPosition);
        m_signals.add(tiltFollowerVelocity);
        m_signals.add(tiltFollowerSuppliedCurrent);
        m_signals.add(tiltFollowerTempCelsius);
        m_signals.add(tiltFollowerForwardSoftLimit);
        m_signals.add(tiltFollowerReverseHardLimit);

        m_signals.add(extendMasterPosition);
        m_signals.add(extendMasterVelocity);
        m_signals.add(extendMasterSuppliedCurrent);
        m_signals.add(extendMasterTempCelsius);
        m_signals.add(extendMasterForwardSoftLimit);
        m_signals.add(extendMasterReverseHardLimit);

        m_signals.add(extendFollowerPosition);
        m_signals.add(extendFollowerVelocity);
        m_signals.add(extendFollowerSuppliedCurrent);
        m_signals.add(extendFollowerTempCelsius);
        m_signals.add(extendFollowerForwardSoftLimit);
        m_signals.add(extendFollowerReverseHardLimit);

        m_signals.add(wristPosition);
        m_signals.add(wristVelocity);
        m_signals.add(wristSuppliedCurrent);
        m_signals.add(wristTempCelsius);
        m_signals.add(wristForwardSoftLimit);
        m_signals.add(wristReverseHardLimit);
    }

    private void configMotors() {
        tiltMaster.getConfigurator().apply(tiltConfig);
        tiltFollower.getConfigurator().apply(tiltConfig);

        extendMaster.getConfigurator().apply(extendConfig);
        extendFollower.getConfigurator().apply(extendConfig);

        wrist.getConfigurator().apply(wristConfig);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_signals.forEach((s) -> s.refresh());

        inputs.tiltRotations = tiltMasterPosition.getValue();
        inputs.tiltVelocityRotPerSec = tiltMasterVelocity.getValue();
        inputs.tiltSuppliedCurrentAmps += tiltMasterSuppliedCurrent.getValue();
        inputs.tiltSuppliedCurrentAmps += tiltFollowerSuppliedCurrent.getValue();
        inputs.tiltHottestTempCelsius = Math.max(tiltMasterTempCelsius.getValue(), tiltFollowerTempCelsius.getValue());
        inputs.tiltForwardSoftLimit = tiltMasterForwardSoftLimit.getValue();
        inputs.tiltReverseHardLimit = tiltMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;

        inputs.extendMeters = extendMasterPosition.getValue();
        inputs.extendVelocityMetersPerSec = extendMasterVelocity.getValue();
        inputs.extendSuppliedCurrentAmps += extendMasterSuppliedCurrent.getValue();
        inputs.extendSuppliedCurrentAmps += extendFollowerSuppliedCurrent.getValue();
        inputs.extendHottestTempCelsius = Math.max(extendMasterTempCelsius.getValue(), extendFollowerTempCelsius.getValue());
        inputs.extendForwardSoftLimit = extendMasterForwardSoftLimit.getValue();
        inputs.extendReverseHardLimit = extendMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;

        inputs.wristRotations = wristPosition.getValue();
        inputs.wristVelocityRotPerSec = wristVelocity.getValue();
        inputs.wristSuppliedCurrentAmps = wristSuppliedCurrent.getValue();
        inputs.wristTempCelsius = wristTempCelsius.getValue();
        inputs.wristReverseHardLimit = wristReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.wristForwardSoftLimit = wristForwardSoftLimit.getValue();
    }

    @Override
    public void setTiltTarget(double rotations) {
        tiltMasterControl.Position = rotations;
    }

    @Override
    public void setTiltFeedForward(double amps) {
        tiltMasterControl.FeedForward = amps;
    }

    @Override
    public void setExtendTarget(double meters) {
        extendMasterControl.Position = meters;
    }

    @Override
    public void setExtendFeedForward(double amps) {
        extendMasterControl.FeedForward = amps;
    }

    @Override
    public void setWristTarget(double rotations) {
        wristControl.Position = rotations;
    }

    @Override
    public void setWristFeedForward(double amps) {
        wristControl.FeedForward = amps;
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
