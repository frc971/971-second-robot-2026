package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.lib.superstructure.*;
import frc.robot.subsystems.Controllers;
import lombok.Getter;
import lombok.Setter;
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

  @Setter @Getter private Goal goal = Goal.NONE;
  @Getter private Mode mode = Mode.DRIVING;

  private boolean prevPovUp = false;
  private boolean currentPovUp = false;

  // default params
  private AngularVelocity flywheelSpeed = RotationsPerSecond.of(15);
  private Angle hoodAngle = Degrees.of(10.0);
  private Angle turretAngle = Degrees.of(45.0);

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

    if (Controllers.XBOX.povLeft().getAsBoolean()) {
      mode = Mode.HOOD;
    } else if (Controllers.XBOX.povRight().getAsBoolean()) {
      mode = Mode.FLYWHEEL;
    }

    currentPovUp = Controllers.XBOX.povUp().getAsBoolean();
    if (currentPovUp && !prevPovUp) {
      mode = Mode.DRIVING;
    }
    prevPovUp = currentPovUp;

    switch (mode) {
      case FLYWHEEL -> {
        if (Controllers.XBOX.leftBumper().getAsBoolean()) {
          flywheelSpeed =
              RotationsPerSecond.of(
                  Math.min(
                      MAX_FLYWHEEL_RPS, flywheelSpeed.plus(FLYWHEEL_STEP).in(RotationsPerSecond)));
        } else if (Controllers.XBOX.rightBumper().getAsBoolean()) {
          flywheelSpeed =
              RotationsPerSecond.of(
                  Math.max(
                      MIN_FLYWHEEL_RPS, flywheelSpeed.minus(FLYWHEEL_STEP).in(RotationsPerSecond)));
        }
      }

      case HOOD -> {
        if (Controllers.XBOX.leftBumper().getAsBoolean()) {
          hoodAngle = Degrees.of(Math.min(MAX_HOOD_DEGREES, hoodAngle.plus(HOOD_STEP).in(Degrees)));
        } else if (Controllers.XBOX.rightBumper().getAsBoolean()) {
          hoodAngle =
              Degrees.of(Math.max(MIN_HOOD_DEGREES, hoodAngle.minus(HOOD_STEP).in(Degrees)));
        }
      }

      case DRIVING -> {}
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
