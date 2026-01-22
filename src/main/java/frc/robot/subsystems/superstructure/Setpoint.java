package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import lombok.Getter;

public class Setpoint {
  @Getter private Optional<Angle> flywheel = Optional.empty();
  @Getter private Optional<Angle> hood = Optional.empty();
  @Getter private Optional<Angle> turret = Optional.empty();
  @Getter private Optional<Voltage> indexer = Optional.empty();
  @Getter private Optional<Voltage> groundRollers = Optional.empty();

  public Setpoint() {}

  public Setpoint withFlywheelDegrees(double degrees) {
    this.flywheel = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withHoodDegrees(double degrees) {
    this.hood = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withTurretDegrees(double degrees) {
    this.turret = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withIndexerVolts(double volts) {
    this.indexer = Optional.of(Volts.of(volts));
    return this;
  }

  public Setpoint withGroundRollersVolts(double volts) {
    this.groundRollers = Optional.of(Volts.of(volts));
    return this;
  }
}
