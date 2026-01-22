package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!!

public class Flywheel extends AngularSubsystem {
  public Flywheel() {
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

    tc.Slot0.kS = 0.0;
    tc.Slot0.kV = 0.0;
    tc.Slot0.kA = 0.0;
    tc.Slot0.kG = 0.0;

    tc.Slot0.kP = 0.0;
    tc.Slot0.kI = 0.0;
    tc.Slot0.kD = 0.0;

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    tc.MotionMagic.MotionMagicCruiseVelocity = 250.0; // TODO: make this reasonable
    tc.MotionMagic.MotionMagicAcceleration = 22.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 44.0 / 12.0; // Motor to output gear ratio

    MotorConfig motorConfig =
        MotorConfig.builder()
            .NAME("Flywheel Lead")
            .ID(8)
            .BUS(new CANBus("Turret"))
            .TALONFX_CONFIG(tc)
            .build();
    return motorConfig;
  }

  public static MotorConfig getFollowerConfig() {
    MotorConfig motorConfig =
        getMotorConfig().toBuilder()
            .NAME("Flywheel Follower")
            .ID(16)
            .FOLLOWER_ALIGNMENT(MotorAlignmentValue.Opposed)
            .build();
    return motorConfig;
  }
}
