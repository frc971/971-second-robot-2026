package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!!
public class FlywheelRight extends AngularSubsystem {
  private static final AngularVelocity STARTUP_DEADBAND = RotationsPerSecond.of(0.5);
  private static final double MAX_PROFILED_ACCELERATION_RPS_PER_SEC = 240.0;
  private static final double MAX_PROFILED_VELOCITY_RPS = 120.0;

  private AngularVelocity profiledVelocity = RotationsPerSecond.zero();

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

    // tc.Slot0.kS = 0.44;
    tc.Slot0.kS = 0.0;
    // tc.Slot0.kV = 0.124;
    tc.Slot0.kV = 0.133;
    tc.Slot0.kA = 0.0;
    tc.Slot0.kG = 0.0;

    // tc.Slot0.kP = 0.55;
    tc.Slot0.kP = 0.4;
    tc.Slot0.kI = 0.0;
    tc.Slot0.kD = 0.0;

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    tc.Slot1.kS = 0.44;
    tc.Slot1.kV = 0.124;
    tc.Slot1.kA = 0.0;
    tc.Slot1.kG = 0.0;

    tc.Slot1.kP = 0.55;
    tc.Slot1.kI = 0.0;
    tc.Slot1.kD = 0.0;

    tc.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    tc.MotionMagic.MotionMagicCruiseVelocity = MAX_PROFILED_VELOCITY_RPS;
    tc.MotionMagic.MotionMagicAcceleration = MAX_PROFILED_ACCELERATION_RPS_PER_SEC;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 35.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 1.0 / 1.0; // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Flywheel Right Lead")
        .ID(41)
        .BUS(new CANBus("Superstructure Bus"))
        .TALONFX_CONFIG(tc)
        .build();
  }

  public static MotorConfig getFollowerConfig() {
    return getMotorConfig().toBuilder()
        .NAME("Flywheel Right Follower")
        .ID(42)
        .FOLLOWER_ALIGNMENT(MotorAlignmentValue.Opposed)
        .build();
  }

  @Override
  public void setVelocity(AngularVelocity goalVelocity) {
    if (goalVelocity.in(RotationsPerSecond) == 0.0) {
      profiledVelocity = RotationsPerSecond.zero();
      setCoast();
      this.goalVelocity = RotationsPerSecond.zero();
      return;
    }

    double startingVelocity = profiledVelocity.in(RotationsPerSecond);
    if (startingVelocity == 0.0
        && !getVelocity().isNear(RotationsPerSecond.zero(), STARTUP_DEADBAND)) {
      startingVelocity = getVelocity().in(RotationsPerSecond);
    }

    double maxDelta = MAX_PROFILED_ACCELERATION_RPS_PER_SEC * Constants.UPDATE_PERIOD.in(Seconds);
    double nextProfiledVelocity =
        MathUtil.clamp(
            goalVelocity.in(RotationsPerSecond),
            startingVelocity - maxDelta,
            startingVelocity + maxDelta);
    profiledVelocity = RotationsPerSecond.of(nextProfiledVelocity);

    if (getVelocity().isNear(RotationsPerSecond.zero(), STARTUP_DEADBAND)
        || profiledVelocity.isNear(RotationsPerSecond.zero(), STARTUP_DEADBAND)) {
      setSlot(1);
    } else {
      setSlot(0);
    }

    super.setVelocity(profiledVelocity);
  }
}
