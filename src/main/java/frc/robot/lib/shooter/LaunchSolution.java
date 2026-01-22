package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
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

  public LaunchSolution(Translation3d fireVelocity) {
    LinearVelocity flywheelSpeed = MetersPerSecond.of(fireVelocity.getNorm());
    Angle hoodAngle = Radians.of(Math.asin(fireVelocity.getZ() / fireVelocity.getNorm()));

    this.shooterData = new ShooterData(hoodAngle, flywheelSpeed);
    this.turretRotation = new Rotation2d(fireVelocity.getX(), fireVelocity.getY());
  }

  public Translation3d fireVector() {
    double speed = shooterData.flywheelSpeed().in(MetersPerSecond);
    double theta = shooterData.hoodAngle().in(Radians);

    double horizontalComponent = speed * Math.cos(theta);
    double x = horizontalComponent * turretRotation.getCos();
    double y = horizontalComponent * turretRotation.getSin();
    double z = speed * Math.sin(theta);

    return new Translation3d(x, y, z);
  }

  public LinearVelocity flywheelSpeed() {
    return this.shooterData.flywheelSpeed();
  }

  public Angle hoodAngle() {
    return this.shooterData.hoodAngle();
  }
}
