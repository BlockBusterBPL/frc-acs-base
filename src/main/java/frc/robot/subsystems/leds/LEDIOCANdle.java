package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LEDIOCANdle implements LEDIO {
    private final CANdle candle;

    public LEDIOCANdle(int id, String bus) {
        candle = new CANdle(id, bus);
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1;
        config.disableWhenLOS = true;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = true;
        config.stripType = CANdle.LEDStripType.GRB;
        config.v5Enabled = true;
        config.vBatOutputMode = CANdle.VBatOutputMode.Off;
        candle.configAllSettings(config);
    }

    public LEDIOCANdle(int id) {
        candle = new CANdle(id);
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.railVoltage5V0 = candle.get5VRailVoltage();
        inputs.busVoltage = candle.getBusVoltage();
        inputs.busCurrentAmps = candle.getCurrent();
        inputs.tempCelsius = candle.getTemperature();
        inputs.vBatModulation = candle.getVBatModulation();
        inputs.hasResetOccurred = candle.hasResetOccurred();
    }

    @Override
    public void setLEDs(int start, int length, int red, int green, int blue) {
        candle.setLEDs(red, green, blue, 0, start, length);
    }

    @Override
    public void animate(Animation animation, int layer) {
        candle.animate(animation, layer);
    }

    @Override
    public void clearAnimation(int layer) {
        candle.clearAnimation(layer);
    }

    @Override
    public void clearLEDs(int start, int length) {
        setLEDs(start, length, 0, 0, 0);
    }

    @Override
    public void setBrightness(double brightness) {
        candle.configBrightnessScalar(brightness);
    }
}
