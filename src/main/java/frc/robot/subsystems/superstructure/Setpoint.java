package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import java.util.Optional;
import lombok.Getter;

@Getter
public class Setpoint {

  public enum Side {
    LEFT,
    RIGHT
  }

  @Getter
  public static class SideConstants {
    private Optional<AngularVelocity> flywheel = Optional.empty();
    private Optional<Angle> hood = Optional.empty();
    private Optional<Angle> turret = Optional.empty();
    private Optional<Voltage> indexer = Optional.empty();

    public SideConstants withFlywheelRPS(double rps) {
      this.flywheel = Optional.of(RotationsPerSecond.of(rps));
      return this;
    }

    public SideConstants withHoodDegrees(double degrees) {
      this.hood = Optional.of(Degrees.of(degrees));
      return this;
    }

    public SideConstants withTurretDegrees(double degrees) {
      this.turret = Optional.of(Degrees.of(degrees));
      return this;
    }

    public SideConstants withIndexerVolts(double volts) {
      this.indexer = Optional.of(Volts.of(volts));
      return this;
    }

    public static SideConstants builder() {
      return new SideConstants();
    }
  }

  private Optional<SideConstants> left = Optional.empty();
  private Optional<SideConstants> right = Optional.empty();

  private Optional<Voltage> groundRollers = Optional.empty();
  private Optional<Angle> groundPivot = Optional.empty();
  private Optional<Voltage> kicker = Optional.empty();

  public Setpoint left(SideConstants left) {
    this.left = Optional.of(left);
    return this;
  }

  public Setpoint right(SideConstants right) {
    this.right = Optional.of(right);
    return this;
  }

  public Setpoint withGroundRollersVolts(double volts) {
    this.groundRollers = Optional.of(Volts.of(volts));
    return this;
  }

  public Setpoint withGroundPivotDegrees(double degrees) {
    this.groundPivot = Optional.of(Degrees.of(degrees));
    return this;
  }

  public static Setpoint builder() {
    return new Setpoint();
  }

  public Setpoint withKickerVolts(double volts) {
    this.kicker = Optional.of(Volts.of(volts));
    return this;
  }

  public SideConstants getSide(Side side) {
    return switch (side) {
      case LEFT -> left.get();
      case RIGHT -> right.get();
    };
  }
}
