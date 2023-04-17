package frc.robot.lib.util;

import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.physics.SwerveDrive;
import frc.robot.lib.swerve.SwerveModuleState;

/**
 * Represents a closed loop output to the drivebase
 */
public class DriveOutput {
    public Rotation2d[] azi_positions; // rad
    public double[] drive_vels; // m/s

    public DriveOutput() {
        this(new Rotation2d[]{
                        Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()},
                new double[]{0, 0, 0, 0});
    }

    public DriveOutput(Rotation2d[] azi_positions, double[] drive_vels) {
        this.azi_positions = azi_positions;
        this.drive_vels = drive_vels;
    }

    public static DriveOutput fromSwerveModuleStates(SwerveModuleState[] state) {
        double[] drive = new double[4];
        Rotation2d[] azi = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            drive[i] = state[i].speedMetersPerSecond;
            azi[i] = state[i].angle;
        }
        return new DriveOutput(azi, drive);
    }
}