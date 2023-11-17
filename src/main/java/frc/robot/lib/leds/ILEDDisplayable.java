package frc.robot.lib.leds;

import com.ctre.phoenix.led.CANdle;

public interface ILEDDisplayable {
    void writePixels(CANdle candle);
}
