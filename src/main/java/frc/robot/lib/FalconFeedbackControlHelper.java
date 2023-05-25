// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.concurrent.TimeoutException;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.hardware.TalonFX;

/** Add your docs here. */
public class FalconFeedbackControlHelper {
    private static final double kTimeout = 0.1;

    private final TalonFX m_motor;
    private final TalonFXConfigurator m_configurator;

    private final Slot0Configs m_pidConfigs;
    private final MotionMagicConfigs m_magicConfigs;

    public FalconFeedbackControlHelper(TalonFX motor, Slot0Configs defaultPID, MotionMagicConfigs defaultMagic) {
        m_motor = motor;
        m_configurator = motor.getConfigurator();

        refreshConfigs();
        m_pidConfigs = defaultPID;
        m_magicConfigs = defaultMagic;
        applyConfigs();
    }

    private void refreshConfigs() {
        refreshConfigs(kTimeout);
    }

    private void refreshConfigs(double timeout) {
        m_configurator.refresh(m_pidConfigs, timeout);
        m_configurator.refresh(m_magicConfigs, timeout);
    }

    private void applyConfigs() {
        applyConfigs(kTimeout);
    }

    private void applyConfigs(double timeout) {
        m_configurator.apply(m_pidConfigs, timeout);
        m_configurator.apply(m_magicConfigs, timeout);
    }

    public void setKP(double kP) {
        refreshConfigs();
        m_pidConfigs.kP = kP;
        applyConfigs();
    }

    public void setKI(double kI) {
        refreshConfigs();
        m_pidConfigs.kI = kI;
        applyConfigs();
    }

    public void setKD(double kD) {
        refreshConfigs();
        m_pidConfigs.kD = kD;
        applyConfigs();
    }

    public void setKV(double kV) {
        refreshConfigs();
        m_pidConfigs.kV= kV;
        applyConfigs();
    }

    public void setMagicVelocity(double magicVelocity) {
        refreshConfigs();
        m_magicConfigs.MotionMagicCruiseVelocity = magicVelocity;
        applyConfigs();
    }

    public void setMagicAcceleration(double magicAcceleration) {
        refreshConfigs();
        m_magicConfigs.MotionMagicAcceleration = magicAcceleration;
        applyConfigs();
    }

    public void setMagicJerk(double magicJerk) {
        refreshConfigs();
        m_magicConfigs.MotionMagicJerk = magicJerk;
        applyConfigs();
    }
}
