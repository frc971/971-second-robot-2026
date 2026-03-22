package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.Set;
import java.util.TreeSet;

/*
 * ShotTable class to manage shooter data (hood angle and flywheel speed) based on distance to target
 * Uses separate interpolation tables for angle and speed
 */
public class ExitSpeedTable {
  // Maps exit linear speed (m/s) to flywheel angular velocity (rot/s)
  /*
   * Separate interpolation tables for hood angle and flywheel speed
   * Key: Linear Speed (in m/s)
   * Value: Angular Speed (in rot/s)
   */
  public final InterpolatingDoubleTreeMap exitSpeedTable = new InterpolatingDoubleTreeMap();

  private final Set<LinearVelocity> speeds = new TreeSet<>();

  public ExitSpeedTable() {}

  public void put(LinearVelocity linearSpeed, AngularVelocity flywheelSpeed) {
    speeds.add(linearSpeed);
    exitSpeedTable.put(linearSpeed.in(MetersPerSecond), flywheelSpeed.in(RotationsPerSecond));
  }

  public AngularVelocity calcAngularVel(LinearVelocity speed) {
    return RotationsPerSecond.of(exitSpeedTable.get(speed.in(MetersPerSecond)));
  }

  public String printSingleLine() {
    StringBuilder sb = new StringBuilder();
    for (LinearVelocity speed : speeds) {
      if (sb.length() > 0) {
        sb.append(" ");
      }
      sb.append(
          String.format(
              "table.put(MetersPerSecond.of(%.5f), RotationsPerSecond.of(%.5f));",
              speed.in(MetersPerSecond), exitSpeedTable.get(speed.in(MetersPerSecond))));
    }
    return sb.toString();
  }
}
