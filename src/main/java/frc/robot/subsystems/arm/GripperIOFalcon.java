package frc.robot.subsystems.arm;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierConfiguration;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ForwardLimitSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitTypeValue;
import com.ctre.phoenixpro.signals.ForwardLimitValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lib.phoenixpro.TalonConfigHelper;
import frc.robot.subsystems.arm.Arm.GameObjectType;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX mGripperMotor;
    private final TalonFXConfiguration mGripperConfig;
    private final DutyCycleOut mGripperControl;

    private final StatusSignalValue<Double> mGripperSpeed;
    private final StatusSignalValue<Double> mGripperSupplyCurrent;
    private final StatusSignalValue<Double> mGripperMotorTemp;
    private final StatusSignalValue<ForwardLimitValue> mGripperForwardLimit;
    private final StatusSignalValue<ReverseLimitValue> mGripperReverseLimit;

    private boolean mConeMode;

    public GripperIOFalcon() {
        mGripperConfig = TalonConfigHelper.getBaseConfig();
        mGripperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.StatorCurrentLimit = 40;
        mGripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.SupplyCurrentLimit = 20;

        mGripperConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        mGripperConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        mGripperMotor = new TalonFX(40, "canivore");
        mGripperMotor.getConfigurator().apply(mGripperConfig);

        mGripperControl = new DutyCycleOut(0, true, false);

        mGripperSpeed = mGripperMotor.getVelocity();
        mGripperSupplyCurrent = mGripperMotor.getSupplyCurrent();
        mGripperMotorTemp = mGripperMotor.getDeviceTemp();
        mGripperForwardLimit = mGripperMotor.getForwardLimit();
        mGripperReverseLimit = mGripperMotor.getReverseLimit();

        mConeMode = false;
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        mGripperSpeed.refresh();
        mGripperSupplyCurrent.refresh();
        mGripperMotorTemp.refresh();
        mGripperForwardLimit.refresh();
        mGripperReverseLimit.refresh();

        inputs.motorSpeedRotationsPerSecond = mGripperSpeed.getValue();
        inputs.suppliedCurrentAmps = mGripperSupplyCurrent.getValue();
        inputs.hottestMotorTempCelsius = mGripperMotorTemp.getValue();

        inputs.cubeInIntake = mGripperReverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.coneInIntake = mGripperForwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }

    @Override
    public void updateOutputs() {
        mGripperMotor.setControl(mGripperControl);
    }

    @Override
    public void setGameObject(GameObjectType object) {
        mConeMode = (object == GameObjectType.CONE);
    }

    @Override
    public void setMotor(double throttle) {
        throttle *= (mConeMode ? -1.0 : 1.0);
        mGripperControl.Output = throttle;
    }
}
