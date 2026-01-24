package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
  public CANcoderConfiguration config;

  public CTREConfigs() {
    config = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    config.MagnetSensor.MagnetOffset = 0.26;
    // config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
  }
}
