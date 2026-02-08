package frc.robot.subsystems.superstructure;

import lombok.Getter;

@Getter
public enum SetpointGoal {
  RESET(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(-18.5)
          .withLeftTurretDegrees(100.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(-18.5)
          .withRightTurretDegrees(100.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)
          .withClimberMeters(0.0)),
  NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(-18.0)
          .withLeftTurretDegrees(0.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(-18.0)
          .withRightTurretDegrees(0.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  EXTEND(Setpoint.builder().withClimberMeters(0.2)),
  RETRACT(Setpoint.builder().withClimberMeters(0.0)),
  INDEX_BOTH(
      Setpoint.builder().withLeftIndexerVolts(6.0).withRightIndexerVolts(6.0).withKickerVolts(6.0)),
  INDEX_RIGHT(
      Setpoint.builder()
          .withKickerVolts(5.0)
          .withLeftIndexerVolts(-5.0)
          .withRightIndexerVolts(5.0)),
  INDEX_LEFT(
      Setpoint.builder()
          .withKickerVolts(5.0)
          .withLeftIndexerVolts(5.0)
          .withRightIndexerVolts(-5.0)),
  OUTTAKE(
      Setpoint.builder()
          .withKickerVolts(-6.0)
          .withLeftIndexerVolts(-6.0)
          .withRightIndexerVolts(-6.0)
          .withGroundRollersVolts(-11.0)
          .withGroundPivotDegrees(0.0)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(10.0)),
  INTAKE_ROLLERS(
      Setpoint.builder()
          .withGroundRollersVolts(11.0)
          .withLeftIndexerVolts(3.0)
          .withRightIndexerVolts(3.0)),
  KILL_RIGHT(
      Setpoint.builder()
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(-18.0)
          .withRightTurretDegrees(0.0)),
  KILL_LEFT(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(-18.0)
          .withLeftTurretDegrees(0.0)),
  MANUAL_MID(
      Setpoint.builder()
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_LEFT(
      Setpoint.builder()
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_RIGHT(
      Setpoint.builder()
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_SHUTTLE_LEFT(
      Setpoint.builder()
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_SHUTTLE_RIGHT(
      Setpoint.builder()
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
