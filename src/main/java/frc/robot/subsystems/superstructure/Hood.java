package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.superstructure.*;

// TODO: change the constants!! and the other numbers

public class Hood extends AngularSubsystem {
  public Hood() {
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

    tc.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile constraints
    tc.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    tc.MotionMagic.MotionMagicAcceleration = 22.0;
    tc.MotionMagic.MotionMagicJerk = 0.0;

    tc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tc.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tc.CurrentLimits.SupplyCurrentLimitEnable = true;
    tc.CurrentLimits.StatorCurrentLimitEnable = true;
    tc.CurrentLimits.SupplyCurrentLimit = 50.0;
    tc.CurrentLimits.StatorCurrentLimit = 100.0;

    tc.Feedback.SensorToMechanismRatio = 112.0 / 1.0; // Motor to output gear ratio

    MotorConfig motorConfig =
        MotorConfig.builder()
            .NAME("Hood")
            .ID(10)
            .BUS(new CANBus("Turret"))
            .TALONFX_CONFIG(tc)
            .LOG_UNIT(Degrees)
            .build();

    return motorConfig;
  }
}
