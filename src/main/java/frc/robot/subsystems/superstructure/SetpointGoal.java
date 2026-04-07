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
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundPivotDegrees(140.0)),
  MANUAL_RESET(
      Setpoint.builder()
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundPivotDegrees(140.0)),
  NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withRollerFloorVolts(0.0)
          .withB2Volts(0.0)
          .withKickerVolts(0.0)
          .withGroundPivotDegrees(120.0)),
  AUTO_NEUTRAL(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withRollerFloorVolts(0.0)),
  SUPERCHARGED(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)
          .withGroundRollersVolts(0.0)
          .withRollerFloorVolts(0.0)),
  AUTO_FLYWHEEL(Setpoint.builder().withLeftFlywheelRPS(0.0).withRightFlywheelRPS(0.0)),
  INDEX(Setpoint.builder().withRollerFloorVolts(8.0).withB2Volts(8.0).withKickerVolts(10.0)),
  OUTTAKE(
      Setpoint.builder()
          .withRollerFloorVolts(-10.0)
          .withGroundRollersVolts(-12.0)
          .withB2Volts(-8.0)),
  UNJAM(Setpoint.builder().withRollerFloorVolts(-10.0).withB2Volts(-8.0).withKickerVolts(-10)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(-7.0)),
  INTAKE_PIVOT_JUICE(Setpoint.builder().withGroundPivotDegrees(50)),
  REVERSE_SHOOTERS(
      Setpoint.builder()
          .withLeftFlywheelRPS(-5.0)
          .withRightFlywheelRPS(-5.0)
          .withB2Volts(-5.0)
          .withKickerVolts(-5.0)),
  AUTO_INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(12.0)),
  INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(10.0)),
  KILL_RIGHT(
      Setpoint.builder()
          .withRightFlywheelRPS(0.0)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(0.0)),
  KILL_LEFT(
      Setpoint.builder()
          .withLeftFlywheelRPS(0.0)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(0.0)),
  MANUAL_SHUTTLE_UP(Setpoint.builder().withRightTurretDegrees(90)),
  MANUAL_SHUTTLE_DOWN(Setpoint.builder().withLeftTurretDegrees(90)),
  MANUAL_SHUTTLE_LEFT(Setpoint.builder()),
  MANUAL_SHUTTLE_RIGHT(Setpoint.builder()),
  MANUAL_UP( // shuttling demo
      Setpoint.builder()
          .withLeftFlywheelRPS(10)
          .withRightFlywheelRPS(10)
          .withLeftHoodInches(0.0)
          .withRightHoodInches(0.0)
          .withLeftTurretDegrees(0)
          .withRightTurretDegrees(0)),
  MANUAL_RIGHT( // right side
      Setpoint.builder()
          .withLeftFlywheelRPS(20)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(-40.527)
          .withRightFlywheelRPS(20)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(-43.925)),
  MANUAL_LEFT( // left side
      Setpoint.builder()
          .withLeftFlywheelRPS(30)
          .withLeftHoodInches(0.0)
          .withLeftTurretDegrees(40.933 - 7.0)
          .withRightFlywheelRPS(30)
          .withRightHoodInches(0.0)
          .withRightTurretDegrees(44.268 - 7.0)),
  MANUAL_DOWN( // up against the hub
      Setpoint.builder()
          .withLeftFlywheelRPS(40)
          .withLeftHoodInches(1.0)
          .withLeftTurretDegrees(0.0)
          .withRightFlywheelRPS(30)
          .withRightHoodInches(1.0)
          .withRightTurretDegrees(0.0));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
