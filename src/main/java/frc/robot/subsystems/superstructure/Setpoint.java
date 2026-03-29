package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import java.util.Optional;
import lombok.Getter;

@Getter
public class Setpoint {
  private Optional<Voltage> groundRollers = Optional.empty();
  private Optional<Angle> groundPivot = Optional.empty();
  private Optional<Distance> climber = Optional.empty();

  private Optional<Distance> rightHood = Optional.empty();
  private Optional<Angle> rightTurret = Optional.empty();
  private Optional<AngularVelocity> rightFlywheel = Optional.empty();

  private Optional<Distance> leftHood = Optional.empty();
  private Optional<Angle> leftTurret = Optional.empty();
  private Optional<AngularVelocity> leftFlywheel = Optional.empty();
  private Optional<Voltage> rollerFloor = Optional.empty();

  public Setpoint withGroundRollersVolts(double volts) {
    this.groundRollers = Optional.of(Volts.of(volts));
    return this;
  }

  public Setpoint withGroundPivotDegrees(double degrees) {
    this.groundPivot = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withLeftFlywheelRPS(double rps) {
    this.leftFlywheel = Optional.of(RotationsPerSecond.of(rps));
    return this;
  }

  public Setpoint withRightFlywheelRPS(double rps) {
    this.rightFlywheel = Optional.of(RotationsPerSecond.of(rps));
    return this;
  }

  public Setpoint withLeftHoodInches(double inches) {
    this.leftHood = Optional.of(Inches.of(inches));
    return this;
  }

  public Setpoint withRightHoodInches(double inches) {
    this.rightHood = Optional.of(Inches.of(inches));
    return this;
  }

  public Setpoint withLeftTurretDegrees(double degrees) {
    this.leftTurret = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withRightTurretDegrees(double degrees) {
    this.rightTurret = Optional.of(Degrees.of(degrees));
    return this;
  }

  public Setpoint withRollerFloorVolts(double volts) {
    this.rollerFloor = Optional.of(Volts.of(volts));
    return this;
  }

  public static Setpoint builder() {
    return new Setpoint();
  }
}
