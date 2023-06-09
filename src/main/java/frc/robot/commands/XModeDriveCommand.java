package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class XModeDriveCommand extends CommandBase {
    private final Drive drive;
    public XModeDriveCommand(Drive drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        drive.stop();
        drive.setXMode(true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setXMode(false);
    }
}
