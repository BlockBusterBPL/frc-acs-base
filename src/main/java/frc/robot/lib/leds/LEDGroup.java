// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

import java.util.Optional;

import com.ctre.phoenix.led.Animation;

/** Add your docs here. */
public class LEDGroup {
    public final int start, length, layer;
    private LEDState lastState;

    public LEDGroup(int start, int length, int layer) {
        this.start = start;
        this.length = length;
        this.layer = layer;
    }

    /**
     * Checks if the state of the LED group needs to be updated, returning the desired state.
     * @param state The given state to compare the current state to.
     * @return The state to change to if required, otherwise empty.
     */
    public Optional<LEDControlData> checkChanged(LEDState state) {
        // only returns value if it needs to be updated
        if (state.equals(lastState)) {
            return Optional.empty();
        } else {
            Optional<Animation> adjustedAnimation = Optional.empty();
            if (state.animation.isPresent()) {
                Animation anim = state.animation.get();
                anim.setLedOffset(start);
                anim.setNumLed(length);
                adjustedAnimation = Optional.of(anim);
            }
            return Optional.of(new LEDControlData(state.color, adjustedAnimation, start, length, layer));
        }
    } 
}
