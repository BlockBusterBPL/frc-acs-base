// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDColor {
    public final int red, green, blue;

    public LEDColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public LEDColor(Color color) {
        this.red = (int) color.red * 255;
        this.green = (int) color.green * 255;
        this.blue = (int) color.blue * 255;
    }
}
