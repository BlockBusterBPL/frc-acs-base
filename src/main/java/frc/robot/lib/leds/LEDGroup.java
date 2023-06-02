// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.leds;

import java.util.Map.Entry;

/** Add your docs here. */
public class LEDGroup {
    public final int start, end;
    private LEDState state;

    public LEDGroup(int start, int end) {
        this.start = start;
        this.end = end;
    }
}
