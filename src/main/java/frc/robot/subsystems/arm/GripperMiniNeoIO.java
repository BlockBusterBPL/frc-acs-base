// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.arm.Arm.GameObjectType;

/** Add your docs here. */
public class GripperMiniNeoIO implements GripperIO {
    private final CANSparkMax mCubeMotor;
    private final CANSparkMax mConeMotor;

    private final RelativeEncoder mCubeEncoder;
    private final RelativeEncoder mConeEncoder;

    private boolean mConeMode;

    public GripperMiniNeoIO() {
        ////////// CUBE MOTOR \\\\\\\\\\
        mCubeMotor = new CANSparkMax(40, MotorType.kBrushless);
        mCubeMotor.restoreFactoryDefaults();
        mCubeMotor.setIdleMode(IdleMode.kBrake);
        mCubeMotor.setInverted(false);
        mCubeMotor.enableVoltageCompensation(12);
        mCubeMotor.setSmartCurrentLimit(20);
        mCubeMotor.setSecondaryCurrentLimit(25);

        mCubeEncoder = mCubeMotor.getEncoder();

        ////////// CONE MOTOR \\\\\\\\\\
        mConeMotor = new CANSparkMax(41, MotorType.kBrushless);
        mConeMotor.restoreFactoryDefaults();
        mConeMotor.setIdleMode(IdleMode.kBrake);
        mConeMotor.setInverted(false);
        mConeMotor.enableVoltageCompensation(12);
        mConeMotor.setSmartCurrentLimit(20);
        mConeMotor.setSecondaryCurrentLimit(25);

        mConeEncoder = mConeMotor.getEncoder();

        mConeMode = false;
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        if (mConeMode) {
            inputs.motorSpeedRotationsPerSecond = mConeEncoder.getVelocity() / 60.0;
        } else {
            inputs.motorSpeedRotationsPerSecond = mCubeEncoder.getVelocity() / 60.0;
        }

        inputs.suppliedCurrentAmps = mCubeMotor.getOutputCurrent() + mConeMotor.getOutputCurrent();

        inputs.hottestMotorTempCelsius = Math.max(mCubeMotor.getMotorTemperature(), mConeMotor.getMotorTemperature());
    }

    @Override
    public void setMotor(double throttle) {
        if (mConeMode) {
            mConeMotor.set(throttle);
            mCubeMotor.set(0);
        } else {
            mConeMotor.set(0);
            mCubeMotor.set(throttle);
        }
    }

    @Override
    public void setGameObject(GameObjectType object) {
        mConeMode = (object == GameObjectType.CONE);
    }
}
