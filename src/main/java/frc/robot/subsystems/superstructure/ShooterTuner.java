package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.lib.superstructure.*;
import frc.robot.subsystems.Controllers;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterTuner {

  public enum Goal {
    ACTIVE,
    NONE
  }

  public enum Mode {
    NONE,
    DRIVING,
    HOOD,
    FLYWHEEL
  }

  private AngularSubsystem flywheel;
  private AngularSubsystem hood;
  private AngularSubsystem turret;

  @AutoLogOutput @Setter @Getter private Goal goal = Goal.NONE;
  @AutoLogOutput @Getter private Mode mode = Mode.DRIVING;

  // default params
  @AutoLogOutput private AngularVelocity flywheelSpeed = RotationsPerSecond.of(15);
  @AutoLogOutput private Angle hoodAngle = Degrees.of(10.0);
  @AutoLogOutput private Angle turretAngle = Degrees.of(45.0);

  // adjustment step sizes (every cycle)
  private static final AngularVelocity FLYWHEEL_STEP = RotationsPerSecond.of(0.5);
  private static final Angle HOOD_STEP = Degrees.of(0.1);

  private static final double MIN_FLYWHEEL_RPS = 0.0;
  private static final double MAX_FLYWHEEL_RPS = 50.0;

  private static final double MIN_HOOD_DEGREES = -18.5;
  private static final double MAX_HOOD_DEGREES = 25.0;

  public ShooterTuner(AngularSubsystem flywheel, AngularSubsystem hood, AngularSubsystem turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  public void periodic() {
    if (goal == Goal.NONE) {
      mode = Mode.NONE;
      return;
    }

    if (Controllers.TOGGLE_HOOD_FLYWHEEL.toggled()) {
      mode = Mode.HOOD;
    } else {
      mode = Mode.FLYWHEEL;
    }

    // TODO: replace with toggle class
    if (Controllers.TOGGLE_DRIVE.toggled()) {
      mode = Mode.FLYWHEEL;
    } else {
      mode = Mode.DRIVING;
    }

    switch (mode) {
      case FLYWHEEL -> {
        if (Controllers.UNDERSHOOT.getAsBoolean()) {
          flywheelSpeed =
              RotationsPerSecond.of(
                  Math.min(
                      MAX_FLYWHEEL_RPS, flywheelSpeed.plus(FLYWHEEL_STEP).in(RotationsPerSecond)));
        } else if (Controllers.OVERSHOOT.getAsBoolean()) {
          flywheelSpeed =
              RotationsPerSecond.of(
                  Math.max(
                      MIN_FLYWHEEL_RPS, flywheelSpeed.minus(FLYWHEEL_STEP).in(RotationsPerSecond)));
        }
      }

      case HOOD -> {
        if (Controllers.UNDERSHOOT.getAsBoolean()) {
          hoodAngle = Degrees.of(Math.min(MAX_HOOD_DEGREES, hoodAngle.plus(HOOD_STEP).in(Degrees)));
        } else if (Controllers.OVERSHOOT.getAsBoolean()) {
          hoodAngle =
              Degrees.of(Math.max(MIN_HOOD_DEGREES, hoodAngle.minus(HOOD_STEP).in(Degrees)));
        }
      }

      case DRIVING, NONE -> {}
    }

    Logger.recordOutput("ShooterTuner/FlywheelSpeed (rps)", flywheelSpeed);
    Logger.recordOutput("ShooterTuner/HoodAngle (deg)", hoodAngle);

    flywheel.setVelocity(flywheelSpeed);
    hood.setPosition(hoodAngle);
    turret.setPosition(turretAngle);
  }

  public boolean freezeDriving() {
    return mode == Mode.HOOD || mode == Mode.FLYWHEEL;
  }
}
