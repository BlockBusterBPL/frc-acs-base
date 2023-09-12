// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

/** Add your docs here. */
public class GripperMiniNeoIO implements GripperIO {
    private final CANSparkMax cube;
    private final CANSparkMax cone;

    private final RelativeEncoder cubeEncoder;
    private final RelativeEncoder coneEncoder;

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
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.motorSpeedRotationsPerSecond[1] = cubeEncoder.getVelocity() / 60.0;
        inputs.motorSpeedRotationsPerSecond[2] = coneEncoder.getVelocity() / 60.0;

        inputs.motorCurrentAmps[1] = cube.getOutputCurrent();
        inputs.motorCurrentAmps[2] = cone.getOutputCurrent();

        inputs.motorTempCelsius[1] = cube.getMotorTemperature();
        inputs.motorTempCelsius[2] = cone.getMotorTemperature();
    }

    @Override
    public void setCube(double throttle) {
        cube.set(throttle);
    }

    @Override
    public void setCone(double throttle) {
        cone.set(throttle);
    }
}
