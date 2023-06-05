package frc.robot.lib.leds;

import java.util.Optional;

import com.ctre.phoenix.led.Animation;

public class LEDControlData extends LEDState {
    public final int start;
    public final int length;
    public final int layer;

    public LEDControlData(LEDColor color, Animation animation, int start, int length, int layer) {
        super(color, animation);
        this.start = start;
        this.length = length;
        this.layer = layer;
    }

    public LEDControlData(LEDColor color, Optional<Animation> animation, int start, int length, int layer) {
        super(color, animation);
        this.start = start;
        this.length = length;
        this.layer = layer;
    }
}
