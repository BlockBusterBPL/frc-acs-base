// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

import java.util.Objects;
import java.util.Optional;

import com.ctre.phoenix.led.Animation;

/** Contains an LED Color and Animation that can be applied to an LED Group */
public class LEDState {
    public final LEDColor color;
    public final Optional<Animation> animation;

    public LEDState(LEDColor color, Animation animation) {
        this.color = color;
        this.animation = Optional.ofNullable(animation);
    }

    public LEDState(LEDColor color, Optional<Animation> animation) {
        this.color = color;
        this.animation = animation;
    }

    public LEDState(LEDColor color) {
        this(color, Optional.empty());
    }

    @Override
    public int hashCode() {
        return Objects.hash(color, animation);
    }
}
