package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  public final AngularVelocity turretVelocity;

  public LaunchSolution() {
    this.shooterData = new ShooterData(Degrees.of(76), RotationsPerSecond.of(0.0));
    this.turretRotation = new Rotation2d();
    this.turretVelocity = RadiansPerSecond.of(0.0);
  }

  public LaunchSolution(ShooterData shooterData, Rotation2d turretRotation) {
    this(shooterData, turretRotation, RadiansPerSecond.of(0.0));
  }

  public LaunchSolution(
      ShooterData shooterData, Rotation2d turretRotation, AngularVelocity turretVelocity) {
    this.shooterData = shooterData;
    this.turretRotation = turretRotation;
    this.turretVelocity = turretVelocity;
  }

  public AngularVelocity flywheelSpeed() {
    return this.shooterData.flywheelSpeed();
  }

  public Angle hoodAngle() {
    return this.shooterData.hoodAngle();
  }

  public AngularVelocity turretVelocity() {
    return this.turretVelocity;
  }
}
