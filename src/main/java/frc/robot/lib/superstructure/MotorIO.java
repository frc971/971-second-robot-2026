package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class MotorIO {
  protected final String name;

  @Getter protected final MotorConfig motorConfig;
  @Getter protected Optional<CANcoderConfig> cancoderConfig = Optional.empty();

  @Getter
  @AutoLogOutput(key = "{name}/Applied Voltage")
  protected Voltage appliedVoltage;

  @Getter
  @AutoLogOutput(key = "{name}/Supply Current")
  protected Current supplyCurrent;

  @Getter
  @AutoLogOutput(key = "{name}/Stator Current")
  protected Current statorCurrent;

  @Getter
  @AutoLogOutput(key = "{name}/Temperature")
  protected Temperature temperature;

  @Getter protected Angle absolutePosition = Rotations.of(0.0);
  @Getter protected Angle position = Rotations.of(0.0);
  @Getter protected AngularVelocity velocity = RotationsPerSecond.of(0.0);

  /* Custom feedforward to be added to the position request */
  @Getter
  @Setter
  @AutoLogOutput(key = "{name}/Feedforward Voltage")
  protected Voltage feedforward = Volts.of(0.0);

  MotorIO(MotorConfig config) {
    this.motorConfig = config;
    this.name = config.NAME();
  }

  public void periodic() {
    Logger.recordOutput(name + "/Position", UnitUtil.toDouble(position, motorConfig.LOG_UNIT()));
    Logger.recordOutput(
        name + "/Absolute Position", UnitUtil.toDouble(absolutePosition, motorConfig.LOG_UNIT()));
    Logger.recordOutput(
        name + "/Velocity", UnitUtil.toDouble(velocity, motorConfig.LOG_UNIT().per(Seconds)));
  }

  /** Set goal voltage */
  public abstract void setVoltage(Voltage goalVoltage);

  /** Set goal velocity */
  public abstract void setVelocity(AngularVelocity goalVelocity);

  /** Set angular goal position */
  public abstract void setPosition(Angle goalPosition);

  /* Set linear goal position.
   *
   * Meant for elevators where gear ratio takes into account rotational to linear conversion.
   */
  public abstract void setPosition(Distance goalPosition);

  public abstract void resetPosition(Angle newPosition);

  public abstract void resetPosition(Distance newPosition);
}
