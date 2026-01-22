package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Controller;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ShooterTuner {

  public enum Goal {
    ACTIVE,
    NONE
  }

  private Flywheel flywheel;
  private Hood hood;
  private Turret turret;

  @Setter @Getter private Goal goal = Goal.NONE;
  @Getter private boolean isFiring = false;

  // default params
  private AngularVelocity flywheelSpeed = RotationsPerSecond.of(50.0);
  private Angle hoodAngle = Degrees.of(30.0);
  private Angle turretAngle = Degrees.of(0.0);

  // adjustment step sizes (every cycle)
  private static final AngularVelocity FLYWHEEL_STEP = RotationsPerSecond.of(1.0);
  private static final Angle HOOD_STEP = Degrees.of(1.0);
  private static final Angle TURRET_STEP = Degrees.of(1.0);

  public ShooterTuner(Flywheel flywheel, Hood hood, Turret turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  public void periodic() {
    if (goal == Goal.NONE) return;

    if (Controller.XBOX.povUp().getAsBoolean()) {
      flywheelSpeed = flywheelSpeed.plus(FLYWHEEL_STEP);
    } else if (Controller.XBOX.povDown().getAsBoolean()) {
      flywheelSpeed =
          RotationsPerSecond.of(
              Math.max(0, flywheelSpeed.minus(FLYWHEEL_STEP).in(RotationsPerSecond)));
    }

    if (Controller.XBOX.povLeft().getAsBoolean()) {
      hoodAngle = hoodAngle.plus(HOOD_STEP);
    } else if (Controller.XBOX.povRight().getAsBoolean()) {
      hoodAngle = hoodAngle.minus(HOOD_STEP);
    }

    if (Controller.XBOX.leftBumper().getAsBoolean()) {
      turretAngle = turretAngle.minus(TURRET_STEP);
    } else if (Controller.XBOX.rightBumper().getAsBoolean()) {
      turretAngle = turretAngle.plus(TURRET_STEP);
    }

    Logger.recordOutput("ShooterTuner/FlywheelSpeed (rps)", flywheelSpeed);
    Logger.recordOutput("ShooterTuner/HoodAngle (deg)", hoodAngle);
    Logger.recordOutput("ShooterTuner/TurretAngle (deg)", turretAngle);
    Logger.recordOutput("ShooterTuner/Firing", isFiring);

    if (isFiring) {
      flywheel.setVelocity(flywheelSpeed);
    } else {
      flywheel.setVelocity(RotationsPerSecond.of(0.0));
    }
    hood.setPosition(hoodAngle);
    turret.setPosition(turretAngle);
  }
}
