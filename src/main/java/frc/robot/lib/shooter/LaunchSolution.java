package frc.robot.lib.shooter;

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
