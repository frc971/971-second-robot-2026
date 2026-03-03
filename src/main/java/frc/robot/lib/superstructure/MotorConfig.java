package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Unit;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
@Builder(toBuilder = true)
@Getter
public class MotorConfig {
  private String NAME;
  private int ID;
  private CANBus BUS;
  private TalonFXConfiguration TALONFX_CONFIG;

  @Builder.Default private boolean FOC = true;

  @Builder.Default private Unit LOG_UNIT = Rotations;

  @Builder.Default private MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Aligned;
}
