package frc.robot.subsystems.superstructure;

// TODO: edit the uhh degrees and voltage setpoints

public enum SetpointGoal {
  RESET(
      new Setpoint()
          .withFlywheelDegrees(0.0)
          .withHoodDegrees(0.0)
          .withTurretDegrees(0.0)
          .withIndexerVolts(0.0)
          .withGroundRollersVolts(0.0)),
  NEUTRAL(
      new Setpoint()
          .withFlywheelDegrees(0.0)
          .withHoodDegrees(10.0)
          .withTurretDegrees(0.0)
          .withIndexerVolts(0.0)),
  INDEX(new Setpoint().withIndexerVolts(10)),
  INTAKE(new Setpoint().withGroundRollersVolts(6.7));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }

  public Setpoint getSetpoint() {
    return setpoint;
  }
}
