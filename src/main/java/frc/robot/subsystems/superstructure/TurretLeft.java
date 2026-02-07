package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.lib.superstructure.*;

// TODO: change the constants...
public class TurretLeft extends AngularSubsystem {
  public static final Angle UPPER_LIMIT = Degrees.of(100);
  public static final Angle LOWER_LIMIT = Degrees.of(-100);
  public static final boolean ENABLE_WRAP = false;
  // If the turret has a goal outside its range BUT it is within this extra buffer it will still
  // clamp to end of its range
  private static final Angle BUFFER = Degrees.of(20);

  public TurretLeft() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.0; // Static friction compensation
    tc.Slot0.kV = 0.0; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.0; // Gravity compensation

    tc.Slot0.kP = 0.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.9;
    tc.MotionMagic.MotionMagicAcceleration = 20.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 185.0 / 16.0; // Motor to output gear ratio

    tc.ClosedLoopGeneral.ContinuousWrap = ENABLE_WRAP;

    return MotorConfig.builder()
        .NAME("Turret Left")
        .ID(-1)
        .BUS(new CANBus("Drivetrain Bus"))
        .LOG_UNIT(Degrees)
        .TALONFX_CONFIG(tc)
        .build();
  }

  public Angle limitAngle(Angle goalAngle) {

    // translating the angle into -180 to 180 range
    double radians = goalAngle.in(Radians);
    Angle normalizedDeg = Radians.of(MathUtil.angleModulus(radians));

    // clamping the angle into -180 to 180 range; also keeps the units degrees
    return normalizedDeg;
  }

  @Override
  public void setPosition(Angle goalPosition) {
    Angle clampedGoalPosition;
    if (ENABLE_WRAP) {
      clampedGoalPosition = limitAngle(goalPosition);
    } else {
      if ((goalPosition.in(Degrees) > UPPER_LIMIT.in(Degrees) + BUFFER.in(Degrees))
          || (goalPosition.in(Degrees) < LOWER_LIMIT.in(Degrees) - BUFFER.in(Degrees))) {
        clampedGoalPosition = UPPER_LIMIT.plus(LOWER_LIMIT).div(2);
      } else {
        clampedGoalPosition =
            Degrees.of(
                MathUtil.clamp(
                    goalPosition.in(Degrees), LOWER_LIMIT.in(Degrees), UPPER_LIMIT.in(Degrees)));
      }
    }
    super.setPosition(clampedGoalPosition);
  }
}
