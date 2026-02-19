package frc.robot.subsystems.superstructure;

import lombok.Getter;

/*
 * Turrets - Positive Voltage = Counterclockwise from top
 * Hoods - Positive Voltage = Higher altitude
 * Indexers - Positive Voltage = Spinning so that balls will go in its corresponding shooter
 * Kicker - Position Voltage = Pushing balls inward
 * Flywheel - Positive Voltage = Push balls out of robot
 */
@Getter
public enum SetpointGoal {
  RESET(
      Setpoint.builder()
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(104.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(104.0)
          .withGroundPivotDegrees(90.0)
          .withClimberMeters(0.0)),
  NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(104.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(104.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(90.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  EXTEND(Setpoint.builder().withClimberMeters(0.2)),
  RETRACT(Setpoint.builder().withClimberMeters(0.0)),
  INDEX_BOTH(
      Setpoint.builder().withLeftIndexerVolts(6.0).withRightIndexerVolts(6.0).withKickerVolts(8.0)),
  INDEX_RIGHT(
      Setpoint.builder()
          .withKickerVolts(8.0)
          .withLeftIndexerVolts(-6.0)
          .withRightIndexerVolts(6.0)),
  INDEX_LEFT(
      Setpoint.builder()
          .withKickerVolts(8.0)
          .withLeftIndexerVolts(6.0)
          .withRightIndexerVolts(-6.0)),
  OUTTAKE(
      Setpoint.builder()
          .withKickerVolts(-6.0)
          .withLeftIndexerVolts(-6.0)
          .withRightIndexerVolts(-6.0)
          .withGroundRollersVolts(-11.0)
          .withGroundPivotDegrees(24.0)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(22.0)),
  INTAKE_ROLLERS(
      Setpoint.builder()
          .withGroundRollersVolts(8.0)
          .withLeftIndexerVolts(0.0)
          .withRightIndexerVolts(0.0)),
  KILL_RIGHT(
      Setpoint.builder()
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)),
  KILL_LEFT(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)),
  MANUAL_MID(
      Setpoint.builder()
          .withLeftTurretDegrees(0.0)
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_LEFT(
      Setpoint.builder()
          .withLeftTurretDegrees(0.0)
          .withLeftHoodDegrees(0)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_RIGHT(
      Setpoint.builder()
          .withLeftTurretDegrees(0)
          .withLeftHoodDegrees(10)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_SHUTTLE_LEFT(
      Setpoint.builder()
          .withLeftTurretDegrees(0)
          .withLeftHoodDegrees(20)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)),
  MANUAL_SHUTTLE_RIGHT(
      Setpoint.builder()
          .withLeftTurretDegrees(0)
          .withLeftHoodDegrees(30)
          .withRightHoodDegrees(10)
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
