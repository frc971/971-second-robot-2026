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

  public static final Angle UPPER_LIMIT = Degrees.of(90);
  public static final Angle LOWER_LIMIT = Degrees.of(-45);
  public static final boolean ENABLE_WRAP = false;

  public TurretLeft() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.39; // Static friction compensation
    tc.Slot0.kV = 6.0; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.0; // Gravity compensation

    tc.Slot0.kP = 3.0; // Proportional gain
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

    tc.Feedback.SensorToMechanismRatio = 63.0 / 1.0; // Motor to output gear ratio

    tc.ClosedLoopGeneral.ContinuousWrap = ENABLE_WRAP;

    return MotorConfig.builder()
        .NAME("Turret Left")
        .ID(30)
        .BUS(new CANBus("Drivetrain Bus"))
        .LOG_UNIT(Degrees)
        .TALONFX_CONFIG(tc)
        .build();
  }

  public Angle limitAngle(Angle goalAngle) {

    // translating the angle into -180 to 180 range
    double radians = goalAngle.in(Radians);
    double normalizedDeg = Radians.of(MathUtil.angleModulus(radians)).in(Degrees);

    // clamping the angle into -180 to 180 range; also keeps the units degrees
    double clampedDeg = MathUtil.clamp(normalizedDeg, -180, 180);
    return (Degrees.of(
        clampedDeg)); // clampedAngle is just goalAngle but forced in the -180 to 180 range
  }

  @Override
  public void setPosition(Angle goalPosition) {
    if (!ENABLE_WRAP
        && (goalPosition.in(Degrees) < LOWER_LIMIT.in(Degrees)
            || goalPosition.in(Degrees) > UPPER_LIMIT.in(Degrees))) return;

    this.goalPosition = (ENABLE_WRAP) ? limitAngle(goalPosition) : goalPosition;
    super.setPosition(goalPosition);
  }
}
