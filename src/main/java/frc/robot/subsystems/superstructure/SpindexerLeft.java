package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constant?

public class SpindexerLeft extends MotorSubsystem {
  SpindexerLeft() {
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

    tc.Feedback.SensorToMechanismRatio = 1.0; // Motor to output gear ratio (small rollers)

    return MotorConfig.builder()
        .NAME("Spindexer Left")
        .ID(32)
        .BUS(new CANBus("Left Superstructure"))
        .TALONFX_CONFIG(tc)
        .build();
  }
}
