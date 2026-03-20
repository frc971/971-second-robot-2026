package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.superstructure.*;

public class Indexer extends MotorSubsystem {
  public Indexer() {
    super(getIO());
  }

  private static MotorIO getIO() {
    if (RobotBase.isReal()) {
      return new MotorWithFollowerTalonFX(
          getLeadMotorConfig(), new MotorConfig[] {getFollowerMotorConfig()});
    } else {
      return new MotorSim(getLeadMotorConfig());
    }
  }

  public static MotorConfig getLeadMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 0.0;
    tc.CurrentLimits.StatorCurrentLimit = 0.0;

    tc.Feedback.SensorToMechanismRatio = 0.0;

    return MotorConfig.builder()
        .NAME("Indexer Lead")
        .ID(0)
        .BUS(new CANBus("rio"))
        .TALONFX_CONFIG(tc)
        .build();
  }

  public static MotorConfig getFollowerMotorConfig() {
    return getLeadMotorConfig().toBuilder()
        .NAME("Indexer Follower")
        .ID(0)
        .BUS(new CANBus("Right Superstructure"))
        .FOLLOWER_ALIGNMENT(MotorAlignmentValue.Opposed)
        .build();
  }
}
