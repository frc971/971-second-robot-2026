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
  AUTO_INDEX(Setpoint.builder().withRollerFloorVolts(11.0).withB2Volts(11.0).withKickerVolts(11.0)),
  AUTO_STOP_INDEXING(
      Setpoint.builder().withRollerFloorVolts(0.0).withKickerVolts(0.0).withB2Volts(0.0)),
  INDEX(Setpoint.builder().withRollerFloorVolts(8.0).withB2Volts(7.0).withKickerVolts(8.0)),
  OUTTAKE(
      Setpoint.builder()
          .withRollerFloorVolts(-10.0)
          .withGroundRollersVolts(-12.0)
          .withKickerVolts(-10)
          .withB2Volts(-8.0)),
  UNJAM(Setpoint.builder().withRollerFloorVolts(-10.0).withB2Volts(-8.0).withKickerVolts(-10)),
  INTAKE_PIVOT(Setpoint.builder().withGroundPivotDegrees(0.0)),
  INTAKE_PIVOT_JUICE(Setpoint.builder().withGroundPivotDegrees(55)),
  REVERSE_SHOOTERS(
      Setpoint.builder()
          .withLeftFlywheelRPS(-5.0)
          .withRightFlywheelRPS(-5.0)
          .withB2Volts(-5.0)
          .withKickerVolts(-5.0)),
  AUTO_INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(10.0)),
  INTAKE_ROLLERS(Setpoint.builder().withGroundRollersVolts(8.0)),
  MANUAL_SHUTTLE_UP(Setpoint.builder()),
  MANUAL_SHUTTLE_DOWN(Setpoint.builder()),
  MANUAL_SHUTTLE_LEFT(Setpoint.builder()),
  MANUAL_SHUTTLE_RIGHT(Setpoint.builder()),
  MANUAL_UP( // shuttling demo
      Setpoint.builder()
          .withLeftFlywheelRPS(46.061)
          .withRightFlywheelRPS(46.038)
          .withLeftHoodInches(0.998)
          .withRightHoodInches(0.997)
          .withLeftTurretDegrees(-2.928)
          .withRightTurretDegrees(1.751)),
  MANUAL_RIGHT( // left side [DONE]
      Setpoint.builder()
          .withLeftFlywheelRPS(59.526)
          .withLeftHoodInches(1.237)
          .withLeftTurretDegrees(-44.187)
          .withRightFlywheelRPS(58.233)
          .withRightHoodInches(1.221)
          .withRightTurretDegrees(-43.045)),
  MANUAL_LEFT( // right side [DONE]
      Setpoint.builder()
          .withLeftFlywheelRPS(58.159)
          .withLeftHoodInches(1.22)
          .withLeftTurretDegrees(42.604)
          .withRightFlywheelRPS(59.444)
          .withRightHoodInches(1.236)
          .withRightTurretDegrees(43.774)),
  MANUAL_DOWN( // up against the hub
      Setpoint.builder()
          .withLeftFlywheelRPS(38.533)
          .withLeftHoodInches(0.735)
          .withLeftTurretDegrees(-5.276)
          .withRightFlywheelRPS(38.486)
          .withRightHoodInches(0.732)
          .withRightTurretDegrees(2.905));

  private final Setpoint setpoint;

  SetpointGoal(Setpoint setpoint) {
    this.setpoint = setpoint;
  }
}
