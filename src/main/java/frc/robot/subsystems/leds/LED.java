package frc.robot.subsystems.leds;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.leds.AnimationBuilder;
import frc.robot.lib.leds.LEDColor;
import frc.robot.lib.leds.LEDControlData;
import frc.robot.lib.leds.LEDGroup;
import frc.robot.lib.leds.LEDState;
import frc.robot.lib.leds.AnimationBuilder.ColorAnimationTypes;

public class LED extends SubsystemBase {
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs;

    private final ArrayList<LEDControlData> statesToUpdate = new ArrayList<>(10);

    private final LEDGroup candle = new LEDGroup(0, 8, 0);
    private final LEDGroup perimeter = new LEDGroup(8, 120, 1);

    private boolean coneMode = false;
    private boolean dsConnected = false;
    private boolean fmsConnected = false;
    private boolean autoRotateAcquire = false;
    private boolean autoRotateHold = false;
    private boolean batteryVoltageLow = false;
    private boolean drivePowerSave = false;
    private boolean teamIndicator1 = false;
    private boolean teamIndicator2 = false;

    public LED(LEDIO io) {
        this.io = io;
        this.inputs = new LEDIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("LEDs", inputs);

        LEDState candleState = new LEDState(LEDColor.kBlack);
        LEDState perimeterState = new LEDState(LEDColor.kBlack);

        LEDColor gamepieceColor = new LEDColor(Color.kPurple);

        // ALL LEDS
        if (drivePowerSave) {

        }

        // PERIMETER LEDS
        if (batteryVoltageLow) {
            // keep gamepiece color, blink perimeter orange
            perimeterState = new LEDState(
                gamepieceColor, 
                AnimationBuilder.generate(ColorAnimationTypes.FLASH, new LEDColor(Color.kOrange), 0.8)
            );
        } else if (teamIndicator1) {
            // keep gamepiece color, blink perimeter red
            perimeterState = new LEDState(
                gamepieceColor, 
                AnimationBuilder.generate(ColorAnimationTypes.FLASH, new LEDColor(Color.kRed), 0.8)
            );
        } else if (teamIndicator2) {
            // keep gamepiece color, blink perimeter blue
            perimeterState = new LEDState(
                gamepieceColor, 
                AnimationBuilder.generate(ColorAnimationTypes.FLASH, new LEDColor(Color.kBlue), 0.8)
            );
        } else if (autoRotateAcquire && !autoRotateHold) {
            // clear gamepiece color, blink perimater gamepiece color
            perimeterState = new LEDState(
                null, 
                AnimationBuilder.generate(ColorAnimationTypes.FLASH, gamepieceColor, 0.93)
            );
        } else if (autoRotateHold) {
            // dim gamepiece color, pulse perimeter gampeice color
            perimeterState = new LEDState(
                null, 
                AnimationBuilder.generate(ColorAnimationTypes.PULSE, gamepieceColor, 0.75)
            );
        } else {
            // static gamepiece color
            perimeterState = new LEDState(gamepieceColor);
        }

        perimeter.checkChanged(perimeterState).ifPresent(statesToUpdate::add);

        // CANDLE BLOCK LEDS
        if (fmsConnected) {
            // solid orange
        } else if (dsConnected) {
            // slow pulse orange with dim orange BG
        } else {
            // fast orange pulse with dim orange BG
        }

        candle.checkChanged(candleState).ifPresent(statesToUpdate::add);

        statesToUpdate.forEach((s) -> {
            io.setLEDs(s.start, s.length, s.color.red, s.color.green, s.color.blue);
            s.animation.ifPresentOrElse((a) -> {io.animate(a, s.layer);}, () -> {io.clearAnimation(s.layer);});
        });

        statesToUpdate.clear();
    }
}
