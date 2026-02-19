package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!! and the other numbers
public class HoodRight extends AngularSubsystem {

  public HoodRight() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.356; // Static friction compensation
    tc.Slot0.kV = 12.6; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.0; // Gravity compensation

    tc.Slot0.kP = 16.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.7;
    tc.MotionMagic.MotionMagicAcceleration = 22.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 25.0;
    tc.CurrentLimits.StatorCurrentLimit = 50.0;

    tc.Feedback.SensorToMechanismRatio = 125.0 / 1.0; // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Hood Right")
        .ID(41)
        .BUS(new CANBus("Right Superstructure"))
        .TALONFX_CONFIG(tc)
        .LOG_UNIT(Degrees)
        .build();
  }
}
