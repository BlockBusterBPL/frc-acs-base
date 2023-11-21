package frc.robot.subsystems.arm;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierConfiguration;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.lib.phoenixpro.TalonConfigHelper;
import frc.robot.subsystems.arm.Arm.GameObjectType;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX mGripperMotor;
    private final TalonFXConfiguration mGripperConfig;
    private final DutyCycleOut mGripperControl;

    private final StatusSignalValue<Double> mGripperSpeed;
    private final StatusSignalValue<Double> mGripperSupplyCurrent;
    private final StatusSignalValue<Double> mGripperMotorTemp;

    private boolean mConeMode;

    public GripperIOFalcon() {
        mGripperConfig = TalonConfigHelper.getBaseConfig();
        mGripperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.StatorCurrentLimit = 40;
        mGripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.SupplyCurrentLimit = 20;

        mGripperMotor = new TalonFX(40, "canivore");
        mGripperMotor.getConfigurator().apply(mGripperConfig);

        mGripperControl = new DutyCycleOut(0, true, false);

        mGripperSpeed = mGripperMotor.getVelocity();
        mGripperSupplyCurrent = mGripperMotor.getSupplyCurrent();
        mGripperMotorTemp = mGripperMotor.getDeviceTemp();

        mConeMode = false;
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        mGripperSpeed.refresh();
        mGripperSupplyCurrent.refresh();
        mGripperMotorTemp.refresh();

        inputs.motorSpeedRotationsPerSecond = mGripperSpeed.getValue();
        inputs.suppliedCurrentAmps = mGripperSupplyCurrent.getValue();
        inputs.hottestMotorTempCelsius = mGripperMotorTemp.getValue();
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
