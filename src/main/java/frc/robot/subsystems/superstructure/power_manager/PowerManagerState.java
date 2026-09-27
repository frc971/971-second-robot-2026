package frc.robot.subsystems.superstructure.power_manager;

public enum PowerManagerState {
  INTAKE(50, 80, 15, 40, 10, 10, 25),
  IDLE(50, 25, 25, 40, 10, 10, 25),
  SCORING(50, 20, 15, 40, 20, 20, 25),
  FEEDING(35, 20, 15, 40, 30, 30, 25),
  OUTTAKE(35, 20, 15, 40, 30, 30, 25),
  AUTONOMOUS(50, 25, 25, 40, 10, 10, 25);

  final double flywheelSupplyCurrent;
  final double groundRollersSupplyCurrent;
  final double groundPivotSupplyCurrent;
  final double hoodSupplyCurrent;
  final double kickerSupplyCurrent;
  final double rollerFloorSupplyCurrent;
  final double b2SupplyCurrent;

  PowerManagerState(
      double flywheelSupplyCurrent,
      double groundRollersSupplyCurrent,
      double groundPivotSupplyCurrent,
      double hoodSupplyCurrent,
      double kickerSupplyCurrent,
      double rollerFloorSupplyCurrent,
      double b2SupplyCurrent) {
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
    this.groundRollersSupplyCurrent = groundRollersSupplyCurrent;
    this.groundPivotSupplyCurrent = groundPivotSupplyCurrent;
    this.hoodSupplyCurrent = hoodSupplyCurrent;
    this.kickerSupplyCurrent = kickerSupplyCurrent;
    this.rollerFloorSupplyCurrent = rollerFloorSupplyCurrent;
    this.b2SupplyCurrent = b2SupplyCurrent;
  }
}
