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
                  .withTurretDegrees(100.0)
                  .withIndexerVolts(0.0))
          .right(
              Setpoint.SideConstants.builder()
                  .withFlywheelRPS(0.0)
                  .withHoodDegrees(-18.5)
                  .withTurretDegrees(100.0)
                  .withIndexerVolts(0.0))
          .withGroundPivotDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)
          .withClimberMeters(0.0)),
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
                  .withIndexerVolts(0.0))
          // should be changed 67 is a placeholder........
          .withGroundPivotDegrees(67.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  INDEX(
      Setpoint.builder()
          .left(Setpoint.SideConstants.builder().withIndexerVolts(6.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(6.0))),
  RIGHT_ONLY(
      Setpoint.builder()
          .withKickerVolts(5.0)
          .left(Setpoint.SideConstants.builder().withIndexerVolts(-5.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(-5.0))),
  LEFT_ONLY(
      Setpoint.builder()
          .withKickerVolts(5.0)
          .left(Setpoint.SideConstants.builder().withIndexerVolts(5.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(5.0))),
  BOTH_SHOOT(
      Setpoint.builder()
          .withKickerVolts(5.0)
          .left(Setpoint.SideConstants.builder().withIndexerVolts(-5.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(5.0))),
  MANUAL_LEFT(Setpoint.builder()),
  MANUAL_RIGHT(Setpoint.builder()),
  SHOOT(Setpoint.builder().withKickerVolts(5.0)),
  INTAKE_ROLLERS(
      Setpoint.builder()
          .withGroundRollersVolts(11.0)
          .left(Setpoint.SideConstants.builder().withIndexerVolts(3.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(3.0))),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(0.0)),
  OUTTAKE(
      Setpoint.builder()
          .left(Setpoint.SideConstants.builder().withIndexerVolts(-6.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(-6.0))
          .withKickerVolts(-6.0)
          .withGroundRollersVolts(-11.0)),
  EXTEND(Setpoint.builder().withClimberMeters(6.0)),
  RETRACT(Setpoint.builder().withClimberMeters(0.0)),
  UNJAM(
      Setpoint.builder()
          .left(Setpoint.SideConstants.builder().withIndexerVolts(-6.0))
          .right(Setpoint.SideConstants.builder().withIndexerVolts(-6.0))
          .withKickerVolts(-6.0));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
