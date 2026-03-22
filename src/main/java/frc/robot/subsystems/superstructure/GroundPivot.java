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
    tc.Slot0.kS = 0.12; // Static friction compensation
    tc.Slot0.kV = 5.5; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.08; // Gravity compensation

    tc.Slot0.kP = 1.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.6;
    tc.MotionMagic.MotionMagicAcceleration = 1.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = false;
    tc.CurrentLimits.StatorCurrentLimitEnable = false;
    tc.CurrentLimits.SupplyCurrentLimit = 0.0;
    tc.CurrentLimits.StatorCurrentLimit = 0.0;

    tc.Feedback.SensorToMechanismRatio = (36.0 / 1.0); // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Ground Pivot")
        .ID(14)
        .BUS(new CANBus("Drivetrain Bus"))
        .LOG_UNIT(Degrees)
        .TALONFX_CONFIG(tc)
        .build();
  }
}
