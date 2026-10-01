package frc.robot.subsystems.superstructure.power_manager;

public enum PowerManagerState {
  DEFAULT(40, 40, 25, 25, 30, 30, 25, 30, 27, 10),
  SHOOTING(50, 20, 15, 40, 20, 20, 25, 30, 20, 10),
  SHUTTLING(60, 20, 15, 40, 15, 15, 25, 30, 27, 10),
  SUPERCHARGED(50, 25, 25, 40, 10, 10, 25, 30, 27, 10);

  public final double flywheelSupplyCurrent;
  public final double groundRollersSupplyCurrent;
  public final double groundPivotSupplyCurrent;
  public final double hoodSupplyCurrent;
  public final double kickerSupplyCurrent;
  public final double rollerFloorSupplyCurrent;
  public final double b2SupplyCurrent;
  public final double turretSupplyCurrent;
  public final double drivetrainSupplyCurrent;
  public final double drivetrainSteerSupplyCurrent;

  PowerManagerState(
      double flywheelSupplyCurrent,
      double groundRollersSupplyCurrent,
      double groundPivotSupplyCurrent,
      double hoodSupplyCurrent,
      double kickerSupplyCurrent,
      double rollerFloorSupplyCurrent,
      double b2SupplyCurrent,
      double turretSupplyCurrent,
      double drivetrainSupplyCurrent,
      double drivetrainSteerSupplyCurrent) {
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
    this.groundRollersSupplyCurrent = groundRollersSupplyCurrent;
    this.groundPivotSupplyCurrent = groundPivotSupplyCurrent;
    this.hoodSupplyCurrent = hoodSupplyCurrent;
    this.kickerSupplyCurrent = kickerSupplyCurrent;
    this.rollerFloorSupplyCurrent = rollerFloorSupplyCurrent;
    this.b2SupplyCurrent = b2SupplyCurrent;
    this.turretSupplyCurrent = turretSupplyCurrent;
    this.drivetrainSupplyCurrent = drivetrainSupplyCurrent;
    this.drivetrainSteerSupplyCurrent = drivetrainSteerSupplyCurrent;
  }
}
