// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import java.util.function.Function;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;

/** Add your docs here. */
public class TalonFXLiveConfigHelper{
    /**
     * A helper function that allows a one-function change of any parameters on a TalonFX Motor Controller.
     * @param talon The TalonFX to configure.
     * @param modifier A function that is provided with the current motor config, and is expected to return
     * the updated config back, where it will be applied to the motor.
     */
    public static void editConfig(TalonFX talon, Function<TalonFXConfiguration, TalonFXConfiguration> modifier) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        talon.getConfigurator().refresh(config, 0.1);
        config = modifier.apply(config);
        talon.getConfigurator().apply(config, 0.1);
    }
}
