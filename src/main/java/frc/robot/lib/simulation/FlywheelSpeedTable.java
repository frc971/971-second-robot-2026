package frc.robot.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.shooter.ExitSpeedTable;
import java.util.Set;
import java.util.TreeSet;

// Maps exit flywheel angular velocity (rot/s) to linear speed (m/s)
public class FlywheelSpeedTable {
  /*
   * Separate interpolation tables for hood angle and flywheel speed
   * Key: Angular Speed (in rot/s)
   * Value: Linear Speed (in m/s)
   */
  public final InterpolatingDoubleTreeMap flywheelSpeedTable = new InterpolatingDoubleTreeMap();

  private final Set<AngularVelocity> angularSpeeds = new TreeSet<>();

  public FlywheelSpeedTable() {}

  public void put(AngularVelocity flywheelSpeed, LinearVelocity linearSpeed) {
    angularSpeeds.add(flywheelSpeed);
    flywheelSpeedTable.put(flywheelSpeed.in(RotationsPerSecond), linearSpeed.in(MetersPerSecond));
  }

  public LinearVelocity calcLinearVel(AngularVelocity speed) {
    return MetersPerSecond.of(flywheelSpeedTable.get(speed.in(RotationsPerSecond)));
  }

  public static FlywheelSpeedTable buildTable(ExitSpeedTable exitSpeedTable) {
    FlywheelSpeedTable table = new FlywheelSpeedTable();
    for (LinearVelocity speed : exitSpeedTable.getSpeeds()) {
      table.put(exitSpeedTable.calcAngularVel(speed), speed);
    }
    return table;
  }

  public String printSingleLine() {
    StringBuilder sb = new StringBuilder();
    for (AngularVelocity speed : angularSpeeds) {
      if (sb.length() > 0) {
        sb.append(" ");
      }
      sb.append(
          String.format(
              "table.put(MetersPerSecond.of(%.5f), RotationsPerSecond.of(%.5f));",
              speed.in(RotationsPerSecond), flywheelSpeedTable.get(speed.in(RotationsPerSecond))));
    }
    return sb.toString();
  }
}
