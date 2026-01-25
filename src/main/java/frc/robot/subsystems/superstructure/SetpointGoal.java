package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SetpointGoal {
  RESET(
      Setpoint.builder()
          .left(
              Setpoint.SideConstants.builder()
                  .withFlywheelRPS(0.0)
                  .withHoodDegrees(-18.5)
                  .withTurretDegrees(90.0)
                  .withIndexerVolts(0.0))
          .right(
              Setpoint.SideConstants.builder()
                  .withFlywheelRPS(0.0)
                  .withHoodDegrees(-18.5)
                  .withTurretDegrees(90.0)
                  .withIndexerVolts(0.0))
          .withGroundPivotDegrees(0.0)
          .withGroundRollersVolts(0.0)),

  NEUTRAL(
      Setpoint.builder()
          .left(
              Setpoint.SideConstants.builder()
                  .withFlywheelRPS(0.0)
                  .withHoodDegrees(-18.0)
                  .withTurretDegrees(0.0)
                  .withIndexerVolts(0.0))
          .right(
              Setpoint.SideConstants.builder()
                  .withFlywheelRPS(0.0)
                  .withHoodDegrees(-18.0)
                  .withTurretDegrees(0.0)
                  .withIndexerVolts(0.0))),

  INDEX(
      Setpoint.builder()
          .left(Setpoint.SideConstants.builder().withIndexerVolts(6.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(6.0))),

  INTAKE(Setpoint.builder().withGroundRollersVolts(11.0));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
