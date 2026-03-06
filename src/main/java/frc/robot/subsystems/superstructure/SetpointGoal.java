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
          .withLeftTurretDegrees(103.5)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(103.5)
          .withGroundPivotDegrees(90.0)
          .withClimberMeters(0.0)),
  NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(101.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(101.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(88.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  AUTO_NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(40.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  SUPERCHARGED(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)
          .withLeftIndexerVolts(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withRightIndexerVolts(0.0)
          .withGroundPivotDegrees(88.0)
          .withGroundRollersVolts(0.0)
          .withKickerVolts(0.0)),
  AUTO_FLYWHEEL(Setpoint.builder().withLeftFlywheelRPS(70.0).withRightFlywheelRPS(70.0)),
  EXTEND(Setpoint.builder().withClimberMeters(0.2)),
  RETRACT(Setpoint.builder().withClimberMeters(0.0)),
  INDEX_BOTH(
      Setpoint.builder()
          .withLeftIndexerVolts(10.0)
          .withRightIndexerVolts(10.0)
          .withKickerVolts(8.0)),
  INDEX_RIGHT(
      Setpoint.builder()
          .withKickerVolts(8.0)
          .withLeftIndexerVolts(-10.0)
          .withRightIndexerVolts(10.0)),
  INDEX_LEFT(
      Setpoint.builder()
          .withKickerVolts(8.0)
          .withLeftIndexerVolts(10.0)
          .withRightIndexerVolts(-10.0)),
  OUTTAKE(
      Setpoint.builder()
          .withKickerVolts(-7.0)
          .withLeftIndexerVolts(0.0)
          .withRightIndexerVolts(0.0)
          .withGroundRollersVolts(-12.0)
          .withGroundPivotDegrees(24.0)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(23.0)),
  AUTO_INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(12.0)),
  INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(10.0)),
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

  MANUAL_SHUTTLE_UP(Setpoint.builder()),
  MANUAL_SHUTTLE_DOWN(Setpoint.builder()),
  MANUAL_SHUTTLE_LEFT(Setpoint.builder()),
  MANUAL_SHUTTLE_RIGHT(Setpoint.builder()),
  MANUAL_UP( // shuttling demo
      Setpoint.builder()
          .withLeftFlywheelRPS(60)
          .withRightFlywheelRPS(60)
          .withLeftHoodDegrees(40)
          .withRightHoodDegrees(40)
          .withLeftTurretDegrees(0)
          .withRightTurretDegrees(0)),
  MANUAL_RIGHT( // right side
      Setpoint.builder()
          .withLeftFlywheelRPS(67.778 + 5.0)
          .withLeftHoodDegrees(43.315)
          .withLeftTurretDegrees(-40.527)
          .withRightFlywheelRPS(60.290 + 5.0)
          .withRightHoodDegrees(38.831)
          .withRightTurretDegrees(-43.925)),
  MANUAL_LEFT( // left side
      Setpoint.builder()
          .withLeftFlywheelRPS(63.488 + 5.0)
          .withLeftHoodDegrees(39.634)
          .withLeftTurretDegrees(40.933)
          .withRightFlywheelRPS(68.156)
          .withRightHoodDegrees(39)
          .withRightTurretDegrees(44.268)),
  MANUAL_DOWN( // up against the hub
      Setpoint.builder()
          .withLeftFlywheelRPS(45)
          .withLeftHoodDegrees(8)
          .withLeftTurretDegrees(-13.152)
          .withRightFlywheelRPS(45)
          .withRightHoodDegrees(8)
          .withRightTurretDegrees(13.871));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
