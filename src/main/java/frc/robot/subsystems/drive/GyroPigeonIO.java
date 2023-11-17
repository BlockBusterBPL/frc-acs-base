// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;

/** Add your docs here. */
public class GyroPigeonIO implements GyroIO {
    // private final Pigeon2 gyro;
    private final Pigeon2Configuration gyroConfig;

    private final StatusSignalValue<Double> yawSignal;
    private final StatusSignalValue<Double> pitchSignal;
    private final StatusSignalValue<Double> rollSignal;

    private final StatusSignalValue<Double> yawVelocitySignal;
    private final StatusSignalValue<Double> pitchVelocitySignal;
    private final StatusSignalValue<Double> rollVelocitySignal;

    private final StatusSignalValue<Double> quatWSignal;
    private final StatusSignalValue<Double> quatXSignal;
    private final StatusSignalValue<Double> quatYSignal;
    private final StatusSignalValue<Double> quatZSignal;

    private final ArrayList<StatusSignalValue<?>> statusSignals;

    public GyroPigeonIO(Pigeon2 gyro) {
        // this.gyro = gyro;
        gyroConfig = new Pigeon2Configuration();
        
        gyro.getConfigurator().apply(gyroConfig);

        yawSignal = gyro.getYaw();
        pitchSignal = gyro.getPitch();
        rollSignal = gyro.getRoll();

        yawVelocitySignal = gyro.getAngularVelocityZ();
        pitchVelocitySignal = gyro.getAngularVelocityY();
        rollVelocitySignal = gyro.getAngularVelocityX();

        quatWSignal = gyro.getQuatW();
        quatXSignal = gyro.getQuatX();
        quatYSignal = gyro.getQuatY();
        quatZSignal = gyro.getQuatZ();

        statusSignals = new ArrayList<>();

        statusSignals.add(yawSignal);
        statusSignals.add(pitchSignal);
        statusSignals.add(rollSignal);
        statusSignals.add(yawVelocitySignal);
        statusSignals.add(pitchVelocitySignal);
        statusSignals.add(rollVelocitySignal);
        statusSignals.add(quatWSignal);
        statusSignals.add(quatXSignal);
        statusSignals.add(quatYSignal);
        statusSignals.add(quatZSignal);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        statusSignals.forEach((v) -> v.refresh());

        inputs.calibrating = false; // pigeon2 does not report calibration status
        inputs.connected = yawSignal.getError().isError(); // for our purposes, if the yaw is errored, we should probably ignore the readings

        inputs.yawAngleRotations = yawSignal.getValue() / 360.0;
        inputs.pitchAngleRotations = pitchSignal.getValue() / 360.0;
        inputs.rollAngleRotations = rollSignal.getValue() / 360.0;

        inputs.yawVelocityRotationsPerSecond = yawVelocitySignal.getValue() / 360.0;
        inputs.pitchVelocityRotationsPerSecond = pitchVelocitySignal.getValue() / 360.0;
        inputs.rollVelocityRotationsPerSecond = rollVelocitySignal.getValue() / 360.0;

        inputs.quaternionW = quatWSignal.getValue();
        inputs.quaternionX = quatXSignal.getValue();
        inputs.quaternionY = quatYSignal.getValue();
        inputs.quaternionZ = quatZSignal.getValue();
    }
}
