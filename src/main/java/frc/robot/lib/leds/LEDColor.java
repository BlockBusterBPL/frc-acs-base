// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

import java.util.Objects;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDColor {
    public static final LEDColor kBlack = new LEDColor(0, 0, 0);
    public static final LEDColor kWhite = new LEDColor(Color.kWhite);
    public static final LEDColor kRed = new LEDColor(Color.kRed);
    public static final LEDColor kOrange = new LEDColor(Color.kOrange);
    public static final LEDColor kYellow = new LEDColor(Color.kYellow);
    public static final LEDColor kGreen = new LEDColor(Color.kGreen);
    public static final LEDColor kBlue = new LEDColor(Color.kBlue);
    public static final LEDColor kPurple = new LEDColor(Color.kPurple);
    public static final LEDColor kPink = new LEDColor(Color.kPink);

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

    public LEDColor multiply(double factor) {
        return new LEDColor((int)(red * factor), (int)(green * factor), (int)(blue * factor));
    }

    @Override
    public int hashCode() {
        return Objects.hash(red, green, blue);
    }
}
