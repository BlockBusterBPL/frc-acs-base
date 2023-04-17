// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.kDriveReduction;
import static frc.robot.Constants.kSteerReduction;
import static frc.robot.Constants.kDriveWheelDiameter;

import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.swerve.SwerveModulePosition;
import frc.robot.lib.swerve.SwerveModuleState;
import frc.robot.lib.util.LoggedTunableNumber;

/** Add your docs here. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    private static final LoggedTunableNumber driveKP = new LoggedTunableNumber("Drive/Module/DriveKP");
    private static final LoggedTunableNumber driveKI = new LoggedTunableNumber("Drive/Module/DriveKI");
    private static final LoggedTunableNumber driveKD = new LoggedTunableNumber("Drive/Module/DriveKD");
    private static final LoggedTunableNumber driveKF = new LoggedTunableNumber("Drive/Module/DriveKF");
    private static final LoggedTunableNumber steerKP = new LoggedTunableNumber("Drive/Module/SteerKP");
    private static final LoggedTunableNumber steerKI = new LoggedTunableNumber("Drive/Module/SteerKI");
    private static final LoggedTunableNumber steerKD = new LoggedTunableNumber("Drive/Module/SteerKD");
    private static final LoggedTunableNumber steerKF = new LoggedTunableNumber("Drive/Module/SteerKF");

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index), inputs);

        if (driveKP.hasChanged(hashCode())) {
            io.setDriveKP(driveKP.get());
        }

        if (driveKI.hasChanged(hashCode())) {
            io.setDriveKI(driveKI.get());
        }

        if (driveKD.hasChanged(hashCode())) {
            io.setDriveKD(driveKD.get());
        }

        if (driveKF.hasChanged(hashCode())) {
            io.setDriveKF(driveKF.get());
        }

        if (steerKP.hasChanged(hashCode())) {
            io.setSteerKP(steerKP.get());
        }

        if (steerKI.hasChanged(hashCode())) {
            io.setSteerKI(steerKI.get());
        }

        if (steerKD.hasChanged(hashCode())) {
            io.setSteerKD(steerKD.get());
        }

        if (steerKF.hasChanged(hashCode())) {
            io.setSteerKF(steerKF.get());
        }

        io.updateOutputs();
    }

    public double getVelocity() {
        return convertRotationsToMeters(inputs.driveVelocityRotPerSec);
    }

    public double getDistance() {
        return convertRotationsToMeters(inputs.driveRotations);
    }

    private double convertRotationsToMeters(double rotations) {
        double wheelCircumference = kDriveWheelDiameter * Math.PI;
        double metersPerMotorRotation = wheelCircumference / kDriveReduction;
        
        return rotations * metersPerMotorRotation;
    }

    private double convertMetersToRotations(double meters) {
        double wheelCircumference = kDriveWheelDiameter * Math.PI;
        double motorRotationsPerMeter = kDriveReduction / wheelCircumference;

        return meters * motorRotationsPerMeter;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.steerPositionRotations);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

}
