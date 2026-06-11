package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

public class Kicker extends MotorSubsystem {
  public Kicker() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 30.0;
    tc.CurrentLimits.StatorCurrentLimit = 80.0;

    tc.Feedback.SensorToMechanismRatio = 0.0;

    return MotorConfig.builder()
        .NAME("Kicker")
        .ID(15)
        .BUS(new CANBus("rio"))
        .TALONFX_CONFIG(tc)
        .FOC(false)
        .build();
  }
}
