package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class UnitUtil {
  public static final double toDouble(Angle angle, Unit unit) {
    if (unit instanceof AngleUnit) {
      return angle.in((AngleUnit) unit);
    } else if (unit instanceof DistanceUnit) {
      return Meters.of(angle.in(Rotations)).in((DistanceUnit) unit);
    } else {
      throw new IllegalArgumentException("unit should be AngleUnit or DistanceUnit");
    }
  }

  public static final double toDouble(AngularVelocity angularVelocity, Unit unit) {
    if (unit instanceof AngularVelocityUnit) {
      return angularVelocity.in((AngularVelocityUnit) unit);
    } else if (unit instanceof LinearVelocityUnit) {
      return MetersPerSecond.of(angularVelocity.in(RotationsPerSecond))
          .in((LinearVelocityUnit) unit);
    } else {
      throw new IllegalArgumentException(
          "unit should be AngularVelocityUnit or LinearVelocityUnit");
    }
  }
}
