package frc.robot.lib.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class AnimationBuilder {

    /** A list of possible CANdle animation types. */
public enum ColorAnimationTypes {
    /**
     * Switches between a foreground color and the background color with no fade time.
     */
    FLASH, 
    
    /**
     * Smoothly fades between the foreground and background color continuously.
     */
    PULSE, 
    
    /**
     * Turns on the LEDs in the group one at a time, starting at the first LED and counting up.
     */
    FLOW_FWD,

    /**
     * Turns on the LEDs in the group one at a time, starting at the last LED and counting down.
     */
    FLOW_REV,

    /** 
     * Sends a pocket of LEDs moving across the strip. Reverses once the start of the pocket reaches the end of the strip. 
     */
    POCKET_FRONT,

    /** 
     * Sends a pocket of LEDs moving across the strip. Reverses once the center of the pocket reaches the end of the strip.
     */
    POCKET_CENTER,

    /** 
     * Sends a pocket of LEDs moving across the strip, Reverses once the end of the pocket reaches the end of the strip. 
    */
    POCKET_BACK,

    /** 
     * Randomly switches LEDs between the specified color and the background color. 
     */
    TWINKLE,

    /** 
     * Randomly switches LEDs to the specified color until they are all on, then turns off all LEDs. 
     */
    TWINKLE_OFF
}
    /**
     * Create a color animation for a group of leds on a CANdle.
     * @param type The animation type.
     * @param color The foreground color of the animation.
     * @param speed The speed of the animation.
     * @param extraParam An additional parameter specified for animation types that use it. Currently only used by the Larson animation.
     */
    public static Animation generate(ColorAnimationTypes type, LEDColor color, double speed, int extraParameter) {
        switch (type) {
            case FLASH:
                return new StrobeAnimation(color.red, color.green, color.blue, 0, speed, 1, 0);
            case PULSE: 
                return new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, 1, 0);
            case FLOW_FWD:
                return new ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, 1, Direction.Forward, 0);
            case FLOW_REV:
                return new ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, 1, Direction.Backward, 0);
            case POCKET_FRONT:
                return new LarsonAnimation(color.red, color.green, color.blue, 0, speed, 1, BounceMode.Front, extraParameter, 0);
            case POCKET_CENTER:
                return new LarsonAnimation(color.red, color.green, color.blue, 0, speed, 1, BounceMode.Center, extraParameter, 0);
            case POCKET_BACK:
                return new LarsonAnimation(color.red, color.green, color.blue, 0, speed, 1, BounceMode.Back, extraParameter, 0);
            case TWINKLE:
                return new TwinkleAnimation(color.red, color.green, color.blue, 0, speed, 1, TwinklePercent.Percent42, 0);
            case TWINKLE_OFF:
                return new TwinkleOffAnimation(color.red, color.green, color.blue, 0, speed, 1, TwinkleOffPercent.Percent42, 0);
            default:
                return new SingleFadeAnimation(0, 0, 0);
        }
    }

    /**
     * Create a color animation for a group of leds on a CANdle.
     * @param type The animation type.
     * @param color The foreground color of the animation.
     * @param speed The speed of the animation.
     */
    public static Animation generate(ColorAnimationTypes type, LEDColor color, double speed) {
        return generate(type, color, speed, 0);
    }
}
