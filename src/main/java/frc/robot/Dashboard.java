package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveWithController;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.SendableTriggerButton;
import frc.robot.lib.dashboard.SupplierWidget;
import frc.robot.lib.dashboard.WidgetConfig;
import frc.robot.lib.dashboard.WidgetSendable;
import frc.robot.lib.dashboard.Alert.SendableAlerts;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.localizer.Localizer;

public class Dashboard {
    // Drive Tab
    public final String driveTabName = "Drive";
    public final WidgetSendable<SendableChooser<Double>> drive_linearSpeedChooser;
    public final WidgetSendable<SendableChooser<Double>> drive_angularSpeedChooser;
    public final SupplierWidget<String> drive_activeDriveController;
    public final SupplierWidget<String> drive_activeKinematicLimits;
    public final WidgetSendable<SendableChooser<Boolean>> drive_enableVision;

    public final WidgetSendable<SendableChooser<Boolean>> drive_allowEncoderCal;
    public final WidgetSendable<SendableTriggerButton> drive_saveEncoderOffsets;
    public final WidgetSendable<SendableTriggerButton> drive_restoreEncoderOffsets;
    public final WidgetSendable<SendableChooser<Boolean>> drive_steerMotorBrake;
    public final WidgetSendable<SendableTriggerButton> drive_refreshOffsets;

    public final WidgetSendable<SendableTriggerButton> drive_mod0Cal;
    public final WidgetSendable<SendableTriggerButton> drive_mod1Cal;
    public final WidgetSendable<SendableTriggerButton> drive_mod2Cal;
    public final WidgetSendable<SendableTriggerButton> drive_mod3Cal;

    public final SupplierWidget<Double> drive_mod0_offset;
    public final SupplierWidget<Double> drive_mod1_offset;
    public final SupplierWidget<Double> drive_mod2_offset;
    public final SupplierWidget<Double> drive_mod3_offset;

    public final SupplierWidget<Double> drive_mod0_position;
    public final SupplierWidget<Double> drive_mod1_position;
    public final SupplierWidget<Double> drive_mod2_position;
    public final SupplierWidget<Double> drive_mod3_position;

    // Setup Tab
    public final String setupTabName = "Setup";
    public final WidgetSendable<SendableAlerts> setup_alerts;
//     public final SupplierWidget<Boolean> setup_redAlliance;
//     public final SupplierWidget<Boolean> setup_blueAlliance;
//     public final WidgetSendable<SendableChooser<String>> setup_autonMode; // TODO: replace this with the actual chooser type

//     public final SupplierWidget<Boolean> setup_batteryOK;
//     public final SupplierWidget<Boolean> setup_canOK;
//     public final SupplierWidget<Boolean> setup_inputsOK;
//     public final SupplierWidget<Boolean> setup_visionOK;
//     public final WidgetSendable<SendableTriggerButton> setup_clearFaults;
    
//     public final SupplierWidget<Boolean> setup_driveOK;
//     public final SupplierWidget<Boolean> setup_armOK;
//     public final SupplierWidget<Boolean> setup_grabberOK;
//     public final SupplierWidget<Boolean> setup_ledsOK;

//     public final WidgetSendable<SendableTriggerButton> setup_scanForFaults;
    

    public Dashboard(Robot robot, RobotContainer container, Drive drive, Arm arm, LED led, Localizer vision) {
        // Drive Tab
        drive_linearSpeedChooser = new WidgetSendable<SendableChooser<Double>>(driveTabName, "Linear Speed Limit",
                DriveWithController.linearSpeedLimitChooser,
                new WidgetConfig(0, 0, 2, 1, BuiltInWidgets.kComboBoxChooser));
        drive_angularSpeedChooser = new WidgetSendable<SendableChooser<Double>>(driveTabName, "Angular Speed Limit",
                DriveWithController.angularSpeedLimitChooser,
                new WidgetConfig(2, 0, 2, 1, BuiltInWidgets.kComboBoxChooser));
        drive_activeDriveController = new SupplierWidget<String>(driveTabName, "Drive Control State", "", () -> drive.getControlState().title,
                new WidgetConfig(0, 1, 2, 1, BuiltInWidgets.kTextView));
        drive_activeKinematicLimits = new SupplierWidget<String>(driveTabName, "Current Kinematic Limits", "", () -> drive.getKinematicLimitsTitle(),
                new WidgetConfig(2, 1, 2, 1, BuiltInWidgets.kTextView));
        drive_enableVision = new WidgetSendable<SendableChooser<Boolean>>(driveTabName, "Use AprilTags",
                vision.visionEnableChooser, new WidgetConfig(0, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        drive_allowEncoderCal = new WidgetSendable<SendableChooser<Boolean>>(driveTabName, "Encoder Updates",
                drive.encoderUpdateChooser, new WidgetConfig(4, 0, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        drive_saveEncoderOffsets = new WidgetSendable<SendableTriggerButton>(driveTabName, "Save Offsets",
                new SendableTriggerButton("Save", () -> {
                    drive.saveEncoderOffsetsToPersist();
                }), new WidgetConfig(4, 1, 1, 1, BuiltInWidgets.kCommand));

        drive_restoreEncoderOffsets = new WidgetSendable<SendableTriggerButton>(driveTabName, "Restore Offsets",
                new SendableTriggerButton("Restore", () -> {
                    drive.updateEncoderOffsetsFromPersist();
                }), new WidgetConfig(5, 1, 1, 1, BuiltInWidgets.kCommand));
        drive_steerMotorBrake = new WidgetSendable<SendableChooser<Boolean>>(driveTabName, "Steer Motor Neutral Mode",
                SwerveModule.steerNeutralMode, new WidgetConfig(4, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        drive_refreshOffsets = new WidgetSendable<SendableTriggerButton>(driveTabName, "Offset Cache",
                new SendableTriggerButton("Refresh", () -> drive.refreshEncoderOffsets()),
                new WidgetConfig(9, 3, 1, 1, BuiltInWidgets.kCommand));

        drive_mod0Cal = new WidgetSendable<SendableTriggerButton>(driveTabName, "Moddule 0",
                new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(0)),
                new WidgetConfig(6, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod1Cal = new WidgetSendable<SendableTriggerButton>(driveTabName, "Moddule 1",
                new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(1)),
                new WidgetConfig(7, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod2Cal = new WidgetSendable<SendableTriggerButton>(driveTabName, "Moddule 2",
                new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(2)),
                new WidgetConfig(8, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod3Cal = new WidgetSendable<SendableTriggerButton>(driveTabName, "Moddule 3",
                new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(3)),
                new WidgetConfig(9, 0, 1, 1, BuiltInWidgets.kCommand));

        drive_mod0_offset = new SupplierWidget<Double>(driveTabName, "Offset 0", 0.0, () -> drive.getCachedEncoderOffsets(0), new WidgetConfig(6, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod1_offset = new SupplierWidget<Double>(driveTabName, "Offset 1", 0.0, () -> drive.getCachedEncoderOffsets(1), new WidgetConfig(7, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod2_offset = new SupplierWidget<Double>(driveTabName, "Offset 2", 0.0, () -> drive.getCachedEncoderOffsets(2), new WidgetConfig(8, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod3_offset = new SupplierWidget<Double>(driveTabName, "Offset 3", 0.0, () -> drive.getCachedEncoderOffsets(3), new WidgetConfig(9, 1, 1, 1, BuiltInWidgets.kTextView));

        drive_mod0_position = new SupplierWidget<Double>(driveTabName, "Position 0", 0.0, () -> drive.getLastEncoderPosition(0), new WidgetConfig(6, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod1_position = new SupplierWidget<Double>(driveTabName, "Position 1", 0.0, () -> drive.getLastEncoderPosition(1), new WidgetConfig(7, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod2_position = new SupplierWidget<Double>(driveTabName, "Position 2", 0.0, () -> drive.getLastEncoderPosition(2), new WidgetConfig(8, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod3_position = new SupplierWidget<Double>(driveTabName, "Position 3", 0.0, () -> drive.getLastEncoderPosition(3), new WidgetConfig(9, 2, 1, 1, BuiltInWidgets.kTextView));

        // Setup Tab
        setup_alerts = new WidgetSendable<Alert.SendableAlerts>(setupTabName, "Master Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(6, 0, 4, 4, "Alerts"));
    }
}
