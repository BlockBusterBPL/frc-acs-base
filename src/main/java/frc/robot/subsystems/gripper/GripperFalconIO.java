package frc.robot.subsystems.gripper;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.lib.phoenixpro.TalonConfigHelper;

public class GripperFalconIO implements GripperIO {
    private final TalonFX gripperMotor;
    private final TalonFXConfiguration gripperConfig;
    private final DutyCycleOut gripperControl;

    private boolean coneMode;

    public GripperFalconIO() {
        gripperConfig = TalonConfigHelper.getBaseConfig();
        gripperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        gripperConfig.CurrentLimits.StatorCurrentLimit = 40;
        gripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripperConfig.CurrentLimits.SupplyCurrentLimit = 20;

        gripperMotor = new TalonFX(40, "canivore");
        gripperMotor.getConfigurator().apply(gripperConfig);

        gripperControl = new DutyCycleOut(0, true, false);

        coneMode = false;
    }

    @Override
    public void updateOutputs() {
        gripperMotor.setControl(gripperControl);
    }

    @Override
    public void setConeMode(boolean coneMode) {
        this.coneMode = coneMode;
    }

    @Override
    public void setMotor(double throttle) {
        throttle *= (coneMode ? -1.0 : 1.0);
        gripperControl.Output = throttle;
    }
}
