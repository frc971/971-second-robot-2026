package frc.robot.subsystems.superstructure;

import lombok.Getter;

// TODO: edit the uhh degrees and voltage setpoints

public enum SetpointGoal {
  RESET(
      new Setpoint()
          .withFlywheelRPS(0.0)
          .withHoodDegrees(-18.5)
          .withTurretDegrees(90.0)
          .withIndexerVolts(0.0)
          .withGroundRollersVolts(0.0)),
  NEUTRAL(
      new Setpoint()
          .withFlywheelRPS(0.0)
          .withHoodDegrees(-18.0)
          .withTurretDegrees(0.0)
          .withIndexerVolts(0.0)
          .withGroundRollersVolts(0.0)),
  INDEX(new Setpoint().withIndexerVolts(10)),
  INTAKE(new Setpoint().withGroundRollersVolts(11.0));

  @Getter private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
