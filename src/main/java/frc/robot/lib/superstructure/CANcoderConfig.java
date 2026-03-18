package frc.robot.lib.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
@Builder(toBuilder = true)
@Getter
public class CANcoderConfig {
  private CANBus BUS;
  private int ID;
  private CANcoderConfiguration CANCODER_CONFIG;
  private double MOTOR_TO_SENSOR_RATIO;
}
