// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

/** Add your docs here. */
public class LEDIOCANdle implements LEDIO {
    private final CANdle candle;

    public LEDIOCANdle(int id, String bus) {
        candle = new CANdle(id, bus);
    }

    @Override
    public void updateInputs(LEDIOInputsAutoLogged inputs) {
        inputs.inputVoltage = candle.getBusVoltage();
    }

    @Override
    public void setLEDs(int r, int g, int b, int w, int startID, int count) {
        candle.setLEDs(r, g, b, w, startID, count);
    }

    @Override
    public void setLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }
}