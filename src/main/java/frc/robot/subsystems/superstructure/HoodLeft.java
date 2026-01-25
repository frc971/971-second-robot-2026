package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!! and the other numbers
public class HoodLeft extends AngularSubsystem {

  public HoodLeft() {
    super(getMotorConfig());
  }

  public static MotorConfig getMotorConfig() {
    TalonFXConfiguration tc = new TalonFXConfiguration();

    // Motion Magic PID and feedforward gains
    tc.Slot0.kS = 0.4; // Static friction compensation
    tc.Slot0.kV = 11.5; // Velocity feedforward
    tc.Slot0.kA = 0.0; // Acceleration feedforward
    tc.Slot0.kG = 0.11; // Gravity compensation

    tc.Slot0.kP = 3.0; // Proportional gain
    tc.Slot0.kI = 0.0; // Integral gain
    tc.Slot0.kD = 0.0; // Derivative gain

    tc.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 0.75;
    tc.MotionMagic.MotionMagicAcceleration = 24.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 112.0 / 1.0; // Motor to output gear ratio

    return MotorConfig.builder()
        .NAME("Hood Left")
        .ID(10)
        .BUS(new CANBus("Turret"))
        .TALONFX_CONFIG(tc)
        .LOG_UNIT(Degrees)
        .build();
  }
}
