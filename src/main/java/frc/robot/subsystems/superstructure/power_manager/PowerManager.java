package frc.robot.subsystems.superstructure.power_manager;

import frc.robot.lib.superstructure.MotorSubsystem;
import frc.robot.subsystems.superstructure.B2;
import frc.robot.subsystems.superstructure.FlywheelLeft;
import frc.robot.subsystems.superstructure.FlywheelRight;
import frc.robot.subsystems.superstructure.GroundPivot;
import frc.robot.subsystems.superstructure.GroundRollers;
import frc.robot.subsystems.superstructure.HoodLeft;
import frc.robot.subsystems.superstructure.HoodRight;
import frc.robot.subsystems.superstructure.Kicker;
import frc.robot.subsystems.superstructure.RollerFloor;

public class PowerManager {
  private final MotorSubsystem flywheelLeft;
  private final MotorSubsystem flywheelRight;
  private final MotorSubsystem groundRollers;
  private final MotorSubsystem groundPivot;
  private final MotorSubsystem hoodLeft;
  private final MotorSubsystem hoodRight;
  private final MotorSubsystem kicker;
  private final MotorSubsystem rollerFloor;
  private final MotorSubsystem b2;

  private PowerManagerState state = PowerManagerState.IDLE;

  public PowerManager(
      FlywheelLeft flywheelLeft,
      FlywheelRight flywheelRight,
      GroundRollers groundRollers,
      GroundPivot groundPivot,
      HoodLeft hoodLeft,
      HoodRight hoodRight,
      Kicker kicker,
      RollerFloor rollerFloor,
      B2 b2) {
    this.flywheelLeft = flywheelLeft;
    this.flywheelRight = flywheelRight;
    this.groundRollers = groundRollers;
    this.groundPivot = groundPivot;
    this.hoodLeft = hoodLeft;
    this.hoodRight = hoodRight;
    this.kicker = kicker;
    this.rollerFloor = rollerFloor;
    this.b2 = b2;
    applyState();
  }

  public PowerManagerState getState() {
    return state;
  }

  private void setState(PowerManagerState newState) {
    if (state == newState) return;
    state = newState;
    applyState();
  }

  public void neutralRequest() {
    setState(PowerManagerState.IDLE);
  }

  public void intakeRequest() {
    setState(PowerManagerState.INTAKE);
  }

  public void feedingRequest() {
    setState(PowerManagerState.FEEDING);
  }

  public void scoringRequest() {
    setState(PowerManagerState.SCORING);
  }

  public void outtakeRequest() {
    setState(PowerManagerState.OUTTAKE);
  }

  public void autonomousRequest() {
    setState(PowerManagerState.AUTONOMOUS);
  }

  private void applyState() {
    flywheelLeft.setSupplyCurrentLimit(state.flywheelSupplyCurrent);
    flywheelRight.setSupplyCurrentLimit(state.flywheelSupplyCurrent);
    groundRollers.setSupplyCurrentLimit(state.groundRollersSupplyCurrent);
    groundPivot.setSupplyCurrentLimit(state.groundPivotSupplyCurrent);
    hoodLeft.setSupplyCurrentLimit(state.hoodSupplyCurrent);
    hoodRight.setSupplyCurrentLimit(state.hoodSupplyCurrent);
    kicker.setSupplyCurrentLimit(state.kickerSupplyCurrent);
    rollerFloor.setSupplyCurrentLimit(state.rollerFloorSupplyCurrent);
    b2.setSupplyCurrentLimit(state.b2SupplyCurrent);
  }
}
