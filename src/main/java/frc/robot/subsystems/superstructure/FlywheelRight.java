package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!!
public class FlywheelRight extends AngularSubsystem {

  public FlywheelRight() {
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

    tc.Slot0.kS = 0.3;
    tc.Slot0.kV = 0.46;
    tc.Slot0.kA = 0.0;
    tc.Slot0.kG = 0.0;

    tc.Slot0.kP = 1.0;
    tc.Slot0.kI = 0.0;
    tc.Slot0.kD = 0.0;

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    tc.MotionMagic.MotionMagicCruiseVelocity = 0.0; // TODO: make this reasonable
    tc.MotionMagic.MotionMagicAcceleration = 0.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 44.0 / 12.0; // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Flywheel Right Lead")
        .ID(8)
        .BUS(new CANBus("Turret"))
        .TALONFX_CONFIG(tc)
        .build();
  }

  public static MotorConfig getFollowerConfig() {
    return getMotorConfig().toBuilder()
        .NAME("Flywheel Right Follower")
        .ID(16)
        .FOLLOWER_ALIGNMENT(MotorAlignmentValue.Opposed)
        .build();
  }
}
