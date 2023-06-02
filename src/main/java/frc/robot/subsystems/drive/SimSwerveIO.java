// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class SimSwerveIO implements SwerveModuleIO {
    private final FlywheelSim driveSim;
    private final FlywheelSim steerSim;

    private final SimpleMotorFeedforward driveFeedForward;
    private final PIDController steerController;

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

    public SimSwerveIO() {
        driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
        steerSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);

        driveFeedForward = new SimpleMotorFeedforward(0, 0.01);
        steerController = new PIDController(10, 0, 0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        steerSim.update(Constants.loopPeriodSecs);

        double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        double driveVelocityMetersPerSec = driveSim.getAngularVelocityRPM() / 60.0 * Constants.kDriveWheelDiameter;
        double driveDistanceDelta = driveVelocityMetersPerSec * Constants.loopPeriodSecs;

        inputs.driveMeters = inputs.driveMeters + driveDistanceDelta;
        inputs.driveVelocityMetersPerSec = driveVelocityMetersPerSec;
        inputs.driveAppliedCurrentAmps = driveSim.getCurrentDrawAmps();
        inputs.driveSuppliedCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelsius = 0.0;

        inputs.steerPositionRotations = turnAbsolutePositionRad;
        inputs.steerVelocityRotPerSec = steerSim.getAngularVelocityRPM() / 60.0;
        inputs.steerAppliedCurrentAmps = steerSim.getCurrentDrawAmps();
        inputs.steerSuppliedCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());
        inputs.steerTempCelsius = 0.0;
    }

    @Override
    public void setDriveSpeedTarget(double speedMetersPerSecond) {
        driveSim.setInputVoltage(driveFeedForward.calculate(speedMetersPerSecond / Constants.kDriveWheelDiameter));
    }

    @Override
    public void setSteerPositionTarget(double steerAngleRotations) {
        steerSim.setInputVoltage(steerController.calculate(turnAbsolutePositionRad / (2.0 * Math.PI), steerAngleRotations));
    }
}
