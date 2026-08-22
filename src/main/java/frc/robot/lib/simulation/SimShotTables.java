package frc.robot.lib.simulation;

import static edu.wpi.first.units.Units.*;

import frc.robot.lib.shooter.ExitSpeedTable;

public class SimShotTables {

  public static FlywheelSpeedTable rightFlywheelSpeedTable() {
    FlywheelSpeedTable table = new FlywheelSpeedTable();

    table.put(RotationsPerSecond.of(0.00000), MetersPerSecond.of(0.00000));

    table.put(RotationsPerSecond.of(36.71875), MetersPerSecond.of(5.45568));
    table.put(RotationsPerSecond.of(40.62500), MetersPerSecond.of(5.91819));
    table.put(RotationsPerSecond.of(45.70313), MetersPerSecond.of(6.61531));
    table.put(RotationsPerSecond.of(50.00000 + 1.0), MetersPerSecond.of(7.27521));
    table.put(RotationsPerSecond.of(55.07813 + 2.0), MetersPerSecond.of(7.94416));
    table.put(RotationsPerSecond.of(60.93750), MetersPerSecond.of(8.60935));
    table.put(RotationsPerSecond.of(89.3950053912), MetersPerSecond.of(13.0));

    return table;
  }

  public static FlywheelSpeedTable leftFlywheelSpeedTable() {
    FlywheelSpeedTable table = new FlywheelSpeedTable();

    table.put(RotationsPerSecond.of(0.00000), MetersPerSecond.of(0.00000));

    table.put(RotationsPerSecond.of(36.71875), MetersPerSecond.of(5.45568));
    table.put(RotationsPerSecond.of(40.62500), MetersPerSecond.of(5.91819));
    table.put(RotationsPerSecond.of(45.70313), MetersPerSecond.of(6.61531));
    table.put(RotationsPerSecond.of(50.00000 + 1.0), MetersPerSecond.of(7.27521));
    table.put(RotationsPerSecond.of(55.07813 + 2.0), MetersPerSecond.of(7.94416));
    table.put(RotationsPerSecond.of(60.93750), MetersPerSecond.of(8.60935));
    table.put(RotationsPerSecond.of(89.3950053912), MetersPerSecond.of(13.0));

    return table;
  }
}
