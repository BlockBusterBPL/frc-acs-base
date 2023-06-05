package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;
public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public double railVoltage5V0;
        public double busCurrentAmps;
        public double busVoltage;
        public double tempCelsius;
        public double vBatModulation;
        public boolean hasResetOccurred;
    }

    public default void updateInputs(LEDIOInputs inputs) {}

    public default void setLEDs(int start, int length, int red, int green, int blue) {}

    public default void animate(Animation animation, int layer) {}
    
    public default void clearAnimation(int layer) {}

    public default void clearLEDs(int start, int length) {}

    public default void setBrightness(double brightness) {}
}
