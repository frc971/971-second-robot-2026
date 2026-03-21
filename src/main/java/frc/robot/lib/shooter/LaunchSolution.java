package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.shooter.ShotTable.ShooterData;

/**
 * Immutable Stores solved parameters for projectile launch - hood angle - launch velocity - turret
 * rotation
 */
public class LaunchSolution {
  public final ShooterData shooterData;
  public final Rotation2d turretRotation;

  public LaunchSolution() {
    this.shooterData = new ShooterData(Degrees.of(76), RotationsPerSecond.of(0.0), Seconds.of(0.0));
    this.turretRotation = new Rotation2d();
  }

  public LaunchSolution(ShooterData shooterData, Rotation2d turretRotation) {
    this.shooterData = shooterData;
    this.turretRotation = turretRotation;
  }

  public AngularVelocity flywheelSpeed() {
    return this.shooterData.flywheelSpeed();
  }

  public Angle hoodAngle() {
    return this.shooterData.hoodAngle();
  }
}
