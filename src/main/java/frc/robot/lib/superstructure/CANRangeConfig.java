package frc.robot.lib.superstructure;

import com.ctre.phoenix6.CANBus;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
@Builder(toBuilder = true)
@Getter
public class CANRangeConfig {
  private String NAME;
  private int ID;
  private CANBus BUS;
}
