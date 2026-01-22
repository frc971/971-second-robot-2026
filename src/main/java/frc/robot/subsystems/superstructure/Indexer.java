package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constant?

public class Indexer extends MotorSubsystem {
  Indexer() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 25.0;
    tc.CurrentLimits.StatorCurrentLimit = 50.0;
    MotorConfig motorConfig =
        MotorConfig.builder()
            .NAME("Indexer")
            .ID(7)
            .BUS(new CANBus("Drivetrain Bus"))
            .TALONFX_CONFIG(tc)
            .build();

    tc.Feedback.SensorToMechanismRatio = 1.0; // Motor to output gear ratio (small rollers)

    return motorConfig;
  }
}
