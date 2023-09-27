// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.configs.TorqueCurrentConfigs;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.lib.StatusCodeHelper;

/** Add your docs here. */
public class FalconCurrentLimitHelper {
    private static final double defaultTimeoutSeconds = 0.1;

    private final TalonFX m_motor;
    private final TalonFXConfigurator m_configurator;

    private final CurrentLimitsConfigs m_normalLimitConfig;
    private final TorqueCurrentConfigs m_FOCLimitConfig;

    public FalconCurrentLimitHelper(TalonFX motor, double defaultLimitAmps) {
        m_motor = motor;
        m_configurator = m_motor.getConfigurator();

        m_normalLimitConfig = new CurrentLimitsConfigs();
        m_FOCLimitConfig = new TorqueCurrentConfigs();

        refreshConfigs(1.0);
    }

    private void refreshConfigs() {
        refreshConfigs(defaultTimeoutSeconds);
    }

    private void refreshConfigs(double timeout) {
        StatusCodeHelper.CheckStatusCode(m_configurator.refresh(m_normalLimitConfig, timeout));
        StatusCodeHelper.CheckStatusCode(m_configurator.refresh(m_FOCLimitConfig, timeout));
    }

    private void applyConfigs() {
        applyConfigs(defaultTimeoutSeconds);
    }

    private void applyConfigs(double timeout) {
        m_configurator.apply(m_normalLimitConfig, timeout);
        m_configurator.apply(m_FOCLimitConfig, timeout);
    }

    public void setCurrentLimit(double amps) {
        refreshConfigs();
        m_normalLimitConfig.StatorCurrentLimitEnable = true;
        m_normalLimitConfig.StatorCurrentLimit = amps;

        m_FOCLimitConfig.PeakForwardTorqueCurrent = amps;
        m_FOCLimitConfig.PeakReverseTorqueCurrent = -amps;
        applyConfigs();
    }
}
