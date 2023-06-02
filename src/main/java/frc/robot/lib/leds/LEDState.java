// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

/** Contains an LED Color and Animation that can be applied to an LED Group */
public class LEDState {
    public final LEDColor color;
    public final LEDAnimation animation;

    public LEDState(LEDColor color, LEDAnimation animation) {
        this.color = color;
        this.animation = animation;
    }
}
