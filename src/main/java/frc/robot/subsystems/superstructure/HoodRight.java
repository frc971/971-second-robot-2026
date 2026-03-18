package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.superstructure.*;

public class HoodRight extends AngularSubsystem {
  private static final Angle KS_ERROR_DEADBAND = Degrees.of(0.5);

  public HoodRight() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.0; // Static friction compensation
    tc.Slot0.kG = 0.0; // Gravity compensation

    tc.Slot0.kP = 0.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    // for sim only
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    tc.MotionMagic.MotionMagicAcceleration = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 0.0;
    tc.CurrentLimits.StatorCurrentLimit = 0.0;

    tc.Feedback.SensorToMechanismRatio = 0.0; // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Hood Right")
        .ID(0)
        .BUS(new CANBus("Right Superstructure"))
        .TALONFX_CONFIG(tc)
        .LOG_UNIT(Degrees)
        .build();
  }

  @Override
  public void setPosition(Angle goalPosition) {
    setFeedforward(calculatePositionFeedforward(goalPosition));
    super.setPositionVoltage(goalPosition);
  }

  private Voltage calculatePositionFeedforward(Angle goalPosition) {
    double kG = io.getMotorConfig().TALONFX_CONFIG().Slot0.kG;
    double kS = io.getMotorConfig().TALONFX_CONFIG().Slot0.kS;

    double positionError = goalPosition.minus(getPosition()).in(Degrees);
    if (Math.abs(positionError) < KS_ERROR_DEADBAND.in(Degrees)) {
      return Volts.of(kG);
    }

    return Volts.of(kG + (kS * Math.signum(positionError)));
  }
}
