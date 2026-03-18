package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class LinearSubsystem extends MotorSubsystem {
  public LinearSubsystem(MotorIO io) {
    super(io);
  }

  public LinearSubsystem(MotorConfig motorConfig) {
    super(motorConfig);
  }

  public LinearSubsystem(MotorConfig motorConfig, CANcoderConfig cancoderConfig) {
    super(motorConfig, cancoderConfig);
  }

  public Distance getLinearPosition() {
    return Meters.of(getPosition().in(Rotations));
  }

  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(getVelocity().in(RotationsPerSecond));
  }
}
