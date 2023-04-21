// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.Alert;
import frc.robot.lib.OverrideSwitches;
import frc.robot.lib.Alert.AlertType;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);

  private final Trigger gyroFail = overrides.driverSwitch(0); // bypass all gyro readings
  private final Trigger derateOverride = overrides.driverSwitch(1); // cancel any drive de-rate states
  private final Trigger pathGenOverride = overrides.driverSwitch(2); // bypass path following

  private final Trigger armForceEnable = overrides.operatorSwitch(0); // bypass arm sanity checks and force manual control
  private final Trigger armCalibrateStart = overrides.operatorSwitch(1); // begin the arm calibration sequence
  private final Trigger overrideArmSafety = overrides.operatorSwitch(2); // run arm at full speed even off FMS
  private final Trigger overrideLedBrightness = overrides.operatorSwitch(3); // full led brightness when off FMS
  private final Trigger ledsIndicateFailed = overrides.operatorSwitch(4); // indicate arm failed on LEDS

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
