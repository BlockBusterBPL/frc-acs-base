// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.lib.Alert;
import frc.robot.lib.OverrideSwitches;
import frc.robot.lib.Alert.AlertType;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.drive.DriveController;
import frc.robot.lib.drive.FieldOrientedDriveController;
import frc.robot.lib.drive.StandardDriveController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSimV1;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FalconSwerveIO;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroNavXIO;
import frc.robot.subsystems.drive.NewSimSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.drive.SwerveModuleIO;

public class RobotContainer {
    private Drive drive;
    private Arm arm;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final OverrideSwitches overrides = new OverrideSwitches(5);

    private final Trigger gyroFail = overrides.driverSwitch(0); // bypass all gyro readings
    private final Trigger derateOverride = overrides.driverSwitch(1); // cancel any drive de-rate states
    private final Trigger pathGenOverride = overrides.driverSwitch(2); // bypass path following

    private final Trigger armForceEnable = overrides.operatorSwitch(0); // bypass arm sanity checks and force manual
                                                                        // control
    private final Trigger armCalibrateStart = overrides.operatorSwitch(1); // begin the arm calibration sequence
    private final Trigger overrideArmSafety = overrides.operatorSwitch(2); // run arm at full speed even off FMS
    private final Trigger overrideLedBrightness = overrides.operatorSwitch(3); // full led brightness when off FMS
    private final Trigger ledsIndicateFailed = overrides.operatorSwitch(4); // indicate arm failed on LEDS

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.WARNING);
    private final Alert overrideDisconnected = new Alert("Override controller disconnected (port 5).", AlertType.INFO);

    private final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
    private final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

    public RobotContainer() {
        configureBindings();

        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_2023C:
                    drive = new Drive(
                            new GyroNavXIO(SPI.Port.kMXP),
                            new FalconSwerveIO(0, "canivore"),
                            new FalconSwerveIO(1, "canivore"),
                            new FalconSwerveIO(2, "canivore"),
                            new FalconSwerveIO(3, "canivore"));
                    break;
                case ROBOT_2023P:
                    break;
                case ROBOT_SIMBOT:
                    drive = new Drive(
                            new GyroIO() {
                            }, // TODO: Gyro Sim
                            new NewSimSwerveIO(),
                            new NewSimSwerveIO(),
                            new NewSimSwerveIO(),
                            new NewSimSwerveIO());
                    arm = new Arm(new ArmIOSimV1());
                    break;
                default:
                    throw new IllegalStateException("Selected robot is not valid.");
            }
        }

        if (drive == null) {
            drive = new Drive(
                    new GyroIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    });
        }

        if (arm == null) {
            arm = new Arm(new ArmIO() {
                
            });
        }

        if (Constants.tuningMode) {
            new Alert("Tuning mode active! This should not be used in competition.", AlertType.INFO).set(true);
        }

        DriveController testController = new FieldOrientedDriveController();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, this::getDriveInputs, () -> testController));
    }

    public void checkControllers() {
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(driver.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
        operatorDisconnected.set(
                !DriverStation.isJoystickConnected(operator.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
        overrideDisconnected.set(!overrides.isConnected());
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private ControllerDriveInputs getDriveInputs() {
        return new ControllerDriveInputs(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()).squareInputs().applyDeadZone(0.05, 0.05, 0.05, 0.08);
    }
}
