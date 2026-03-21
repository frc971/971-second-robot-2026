package frc.robot.subsystems.superstructure;

import lombok.Getter;

/*
 * Turrets - Positive Voltage = Counterclockwise from top
 * Hoods - Positive Voltage = Higher altitude
 * Indexer - Positive Voltage = Feeding notes forward
 * Flywheel - Positive Voltage = Push balls out of robot
 */
@Getter
public enum SetpointGoal {
  RESET(
      Setpoint.builder()
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(105)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(107)
          .withGroundPivotDegrees(127.0)
          .withClimberMeters(0.0)),
  MANUAL_RESET(
      Setpoint.builder()
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(104)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(106)
          .withGroundPivotDegrees(90.0)
          .withClimberMeters(0.0)),
  NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withIndexerVolts(0.0)),
  AUTO_NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withIndexerVolts(0.0)),
  SUPERCHARGED(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodDegrees(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withIndexerVolts(0.0)),
  AUTO_FLYWHEEL(Setpoint.builder().withLeftFlywheelRPS(0.0).withRightFlywheelRPS(0.0)),
  EXTEND(Setpoint.builder().withClimberMeters(0.2)),
  RETRACT(Setpoint.builder().withClimberMeters(0.0)),
  INDEX(Setpoint.builder().withIndexerVolts(10.0)),
  OUTTAKE(Setpoint.builder().withIndexerVolts(-10.0).withGroundRollersVolts(-12.0)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(0.0)),
  INTAKE_PIVOT_JUICE(Setpoint.builder().withGroundPivotDegrees(45.0)),
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
          .withLeftTurretDegrees(40.933 - 7.0)
          .withRightFlywheelRPS(68.156)
          .withRightHoodDegrees(39)
          .withRightTurretDegrees(44.268 - 7.0)),
  MANUAL_DOWN( // up against the hub
      Setpoint.builder()
          .withLeftFlywheelRPS(42.0)
          .withLeftHoodDegrees(11.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(42.0)
          .withRightHoodDegrees(11.0)
          .withRightTurretDegrees(0.0));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
