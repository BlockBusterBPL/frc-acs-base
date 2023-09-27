package frc.robot.lib.phoenixpro;

import java.util.function.Function;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;

public class CANcoderLiveConfigHelper {
    public static void editConfig(CANcoder encoder, Function<CANcoderConfiguration, CANcoderConfiguration> mutator) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(config, 0.1);
        config = mutator.apply(config);
        encoder.getConfigurator().apply(config, 0.1);
    }
}
