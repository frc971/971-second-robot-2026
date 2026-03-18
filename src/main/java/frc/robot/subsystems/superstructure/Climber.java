package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!! and the other numbers

public class Climber extends LinearSubsystem {
  private static final Distance KS_ERROR_DEADBAND = Meters.of(0.05);

  public Climber() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.0; // Static friction compensation
    tc.Slot0.kV = 0.0; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.0; // Gravity compensation, negative

    tc.Slot0.kP = 0.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.1;
    tc.MotionMagic.MotionMagicAcceleration = 1.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio =
        100.0 / 1.0; // Not a real gear ratio, put this in so position values aren't obscene

    return MotorConfig.builder()
        .NAME("Climber")
        .ID(17)
        .BUS(new CANBus("Left Superstructure"))
        .TALONFX_CONFIG(tc)
        .LOG_UNIT(Meters)
        .build();
  }

  @Override
  public void setPosition(Distance goalPosition) {
    setFeedforward(calculatePositionFeedforward(goalPosition));
    super.setPositionVoltage(goalPosition);
  }

  private Voltage calculatePositionFeedforward(Distance goalPosition) {
    double kG = io.getMotorConfig().TALONFX_CONFIG().Slot0.kG;
    double kS = io.getMotorConfig().TALONFX_CONFIG().Slot0.kS;

    double positionError = goalPosition.minus(getLinearPosition()).in(Meters);
    if (Math.abs(positionError) < KS_ERROR_DEADBAND.in(Meters)) {
      return Volts.of(kG);
    }

    return Volts.of(kG + (kS * Math.signum(positionError)));
  }
}
