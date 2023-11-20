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
    private final CANSparkMax cube;
    private final CANSparkMax cone;

    private final RelativeEncoder cubeEncoder;
    private final RelativeEncoder coneEncoder;

    private boolean coneMode;

    public GripperMiniNeoIO() {
        ////////// CUBE MOTOR \\\\\\\\\\
        cube = new CANSparkMax(40, MotorType.kBrushless);
        cube.restoreFactoryDefaults();
        cube.setIdleMode(IdleMode.kBrake);
        cube.setInverted(false);
        cube.enableVoltageCompensation(12);
        cube.setSmartCurrentLimit(20);
        cube.setSecondaryCurrentLimit(25);

        cubeEncoder = cube.getEncoder();

        ////////// CONE MOTOR \\\\\\\\\\
        cone = new CANSparkMax(41, MotorType.kBrushless);
        cone.restoreFactoryDefaults();
        cone.setIdleMode(IdleMode.kBrake);
        cone.setInverted(false);
        cone.enableVoltageCompensation(12);
        cone.setSmartCurrentLimit(20);
        cone.setSecondaryCurrentLimit(25);

        coneEncoder = cone.getEncoder();

        coneMode = false;
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        if (coneMode) {
            inputs.motorSpeedRotationsPerSecond = coneEncoder.getVelocity() / 60.0;
        } else {
            inputs.motorSpeedRotationsPerSecond = cubeEncoder.getVelocity() / 60.0;
        }

        inputs.suppliedCurrentAmps = cube.getOutputCurrent() + cone.getOutputCurrent();

        inputs.hottestMotorTempCelsius = Math.max(cube.getMotorTemperature(), cone.getMotorTemperature());
    }

    @Override
    public void setMotor(double throttle) {
        if (coneMode) {
            cone.set(throttle);
            cube.set(0);
        } else {
            cone.set(0);
            cube.set(throttle);
        }
    }

    @Override
    public void setGameObject(GameObjectType object) {
        this.coneMode = (object == GameObjectType.CONE);
    }
}
