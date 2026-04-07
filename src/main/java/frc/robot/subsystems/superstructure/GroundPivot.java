package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constants...

public class GroundPivot extends AngularSubsystem {

  public GroundPivot() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.37; // Static friction compensation
    tc.Slot0.kV = 5.0; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = -0.05; // Gravity compensation

    tc.Slot0.kP = 2.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = .8;
    tc.MotionMagic.MotionMagicAcceleration = 10.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 35.0;
    tc.CurrentLimits.StatorCurrentLimit = 80.0;

    tc.Feedback.SensorToMechanismRatio =
        ((52 / 8) * (56 / 16.0) * (18.0 / 9.0)); // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Ground Pivot")
        .ID(14)
        .BUS(new CANBus("rio"))
        .LOG_UNIT(Degrees)
        .TALONFX_CONFIG(tc)
        .build();
  }
}
