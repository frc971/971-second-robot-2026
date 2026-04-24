package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.superstructure.*;

public class GroundRollers extends MotorSubsystem {
  public GroundRollers() {
    super(getIO());
  }

  private static MotorIO getIO() {
    if (RobotBase.isReal()) {
      return new MotorWithFollowerTalonFX(
          getMotorConfig(), new MotorConfig[] {getFollowerConfig()});
    } else {
      return new MotorSim(getMotorConfig());
    }
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 20.0;
    tc.CurrentLimits.StatorCurrentLimit = 80.0;

    return MotorConfig.builder()
        .NAME("Ground Roller Lead")
        .ID(20)
        .BUS(new CANBus("rio"))
        .TALONFX_CONFIG(tc)
        .FOC(false)
        .build();
  }

  public static MotorConfig getFollowerConfig() {
    return getMotorConfig().toBuilder()
        .NAME("Ground Roller Follower")
        .ID(17)
        .FOLLOWER_ALIGNMENT(MotorAlignmentValue.Opposed)
        .build();
  }
}
