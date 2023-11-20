package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateTracker;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.leds.LEDStateContainer;
import frc.robot.lib.leds.TimedLEDState;

public class LED extends SubsystemBase {
    public enum WantedAction {
        DISPLAY_BATTERY_LOW,
        DISPLAY_GOOD_BATTERY,
        DISPLAY_NOT_HOMED,
        DISPLAY_DELIVERY,
        DISPLAY_VISION,
        DISPLAY_CONFIGURE_FAULT,
        OFF
    }

    private enum SystemState {
        DISPLAYING_BATTERY_LOW,
        DISPLAYING_CONFIGURE_FAULT,
        DISPLAYING_GOOD_BATTERY,
        DISPLAYING_NOT_HOMED,
        DISPLAYING_DELIVERY,
        DISPLAYING_VISION,
        OFF
    }

    private LEDIO ledIO;
    private LEDIOInputsAutoLogged ledIOInputs;

    private SystemState mSystemState = SystemState.OFF;
    private static WantedAction mWantedAction = WantedAction.OFF;

    private LEDStateContainer mDesiredLEDState = new LEDStateContainer();

    private static TimedLEDState mDeliveryLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    private double mStateStartTime;

    public LED(LEDIO ledIO) {
        this.ledIO = ledIO;
        ledIOInputs = new LEDIOInputsAutoLogged();

        mStateStartTime = Timer.getFPGATimestamp();
    }

    public static void setDeliveryLEDState(TimedLEDState ledState) {
        mDeliveryLEDState = ledState;
    }

    public static void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void periodic() {
        ledIO.updateInputs(ledIOInputs);

        double timestamp = Timer.getFPGATimestamp();
        SystemState newState = getStateTransition();
        if (mSystemState != newState) {
            mSystemState = newState;
            mStateStartTime = timestamp;
        }

        Logger.getInstance().recordOutput("LEDs/SystemState", mSystemState.name());

        double timeInState = timestamp - mStateStartTime;
        switch(mSystemState) {
            case DISPLAYING_BATTERY_LOW:
                setBatteryLowCommand(timeInState);
                break;
            case DISPLAYING_CONFIGURE_FAULT:
                setConfigureFault(timeInState);
                break;
            case DISPLAYING_GOOD_BATTERY:
                setGoodBattery(timeInState);
                break;
            case DISPLAYING_NOT_HOMED:
                setNotHomedCommand(timeInState);
                break;
            case DISPLAYING_DELIVERY:
                setDeliveryLEDCommand(timeInState);
                break;
            case DISPLAYING_VISION:
                setDisplayingVision(timeInState);
                break;
            case OFF:
                setOffCommand(timeInState);
                break;
            default:
                break;

        }
        mDesiredLEDState.writePixels(ledIO);
    }

    private void setDeliveryLEDCommand(double timeInState) {
        mDeliveryLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setOffCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticOff.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setConfigureFault(double timeInState) {
        TimedLEDState.BlinkingLEDState.kConfigureFail.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setDisplayingVision(double timeInState) {
        // Limelight limelight = Limelight.getInstance();
        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        if (DriverStation.isDisabled()) {
            if (results.targetingResults.targets_Fiducials.length < 1) {
                // If in disabled, and don't have target show yellow rapid blink.
                TimedLEDState.BlinkingLEDState.kVisionMissing.getCurrentLEDState(mDesiredLEDState, timeInState);
            } else {
                // Otherwise, go green.
                TimedLEDState.BlinkingLEDState.kVisionPresent.getCurrentLEDState(mDesiredLEDState, timeInState);
            }
        } else {
            // If we are in auto, show when limelight goes active.
            if (!RobotStateTracker.getInstance().getAutoAlignActive()) {
                TimedLEDState.StaticLEDState.kVisionDisabled.getCurrentLEDState(mDesiredLEDState, timeInState);
            } else {
                TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
            }
        }
        // TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setNotHomedCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticNotHomed.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setGoodBattery(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setBatteryLowCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticBatteryLow.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private boolean configure_fault = false;
    public synchronized void setConfigureFault(boolean fault){
        configure_fault = fault;
    }

    private SystemState getStateTransition() {
        if (configure_fault) return SystemState.DISPLAYING_CONFIGURE_FAULT;
        switch (mWantedAction) {
            case DISPLAY_DELIVERY:
                return SystemState.DISPLAYING_DELIVERY;
            case DISPLAY_GOOD_BATTERY:
                return SystemState.DISPLAYING_GOOD_BATTERY;
            case DISPLAY_BATTERY_LOW:
                return SystemState.DISPLAYING_BATTERY_LOW; 
            case DISPLAY_NOT_HOMED:
                return SystemState.DISPLAYING_NOT_HOMED;
            case DISPLAY_VISION:
                return SystemState.DISPLAYING_VISION;
            case OFF:
                return SystemState.OFF;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.OFF;
        }
    }
}
