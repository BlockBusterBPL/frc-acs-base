// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.kDriveReduction;
import static frc.robot.Constants.kSteerReduction;
import static frc.robot.Constants.kDriveWheelDiameter;

import frc.robot.lib.util.LoggedTunableBoolean;
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

    private static final LoggedDashboardBoolean steerNeutralMode = new LoggedDashboardBoolean("Drive/Module/SteerNeutral");
    private static boolean steerIsNeutral = false;

    public static boolean getSteerNeutralMode() {
        return steerIsNeutral;
    }

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

        if (steerNeutralMode.get() != steerIsNeutral) {
            io.setSteerBrakeMode(!steerNeutralMode.get());
            steerIsNeutral = steerNeutralMode.get();
        }

        io.updateOutputs();
    }

    public double getVelocity() {
        return inputs.driveVelocityMetersPerSec;
    }

    public double getDistance() {
        return inputs.driveMeters;
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

    public SwerveModuleState setState(SwerveModuleState state) {
        // TODO: try removing optimization from module code, as 254 swerve controller should do that already.
        var optimizedState = SwerveModuleState.optimize(state, getAngle());
        io.setDriveSpeedTarget(optimizedState.speedMetersPerSecond);
        io.setSteerPositionTarget(optimizedState.angle.getRotations());
        return optimizedState;
    }

    public void stop() {
        setState(new SwerveModuleState());
    }

    public void setDriveBrakeMode(boolean brake) {
        io.setDriveBrakeMode(brake);
    }

    public void setSteerBrakeMode(boolean brake) {
        io.setSteerBrakeMode(brake);
    }

    public double getTotalCurrent() {
        return inputs.driveSuppliedCurrentAmps + inputs.steerSuppliedCurrentAmps;
    }
}
