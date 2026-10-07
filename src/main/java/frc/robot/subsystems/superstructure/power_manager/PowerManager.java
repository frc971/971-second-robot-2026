package frc.robot.subsystems.superstructure.power_manager;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.B2;
import frc.robot.subsystems.superstructure.FlywheelLeft;
import frc.robot.subsystems.superstructure.FlywheelRight;
import frc.robot.subsystems.superstructure.GroundPivot;
import frc.robot.subsystems.superstructure.GroundRollers;
import frc.robot.subsystems.superstructure.HoodLeft;
import frc.robot.subsystems.superstructure.HoodRight;
import frc.robot.subsystems.superstructure.Kicker;
import frc.robot.subsystems.superstructure.RollerFloor;
import frc.robot.subsystems.superstructure.ShooterHandler;
import frc.robot.subsystems.superstructure.TurretLeft;
import frc.robot.subsystems.superstructure.TurretRight;
import org.littletonrobotics.junction.Logger;

public class PowerManager {
  private final CommandSwerveDrivetrain drivetrain;
  private final FlywheelLeft flywheelLeft;
  private final FlywheelRight flywheelRight;
  private final GroundRollers groundRollers;
  private final GroundPivot groundPivot;
  private final HoodLeft hoodLeft;
  private final HoodRight hoodRight;
  private final Kicker kicker;
  private final RollerFloor rollerFloor;
  private final B2 b2;
  private final TurretLeft turretLeft;
  private final TurretRight turretRight;
  private final ShooterHandler shooterHandlerLeft;
  private final ShooterHandler shooterHandlerRight;
  private PowerManagerState state = PowerManagerState.DEFAULT;

  public PowerManager(
      CommandSwerveDrivetrain drivetrain,
      FlywheelLeft flywheelLeft,
      FlywheelRight flywheelRight,
      GroundRollers groundRollers,
      GroundPivot groundPivot,
      HoodLeft hoodLeft,
      HoodRight hoodRight,
      Kicker kicker,
      RollerFloor rollerFloor,
      B2 b2,
      TurretLeft turretLeft,
      TurretRight turretRight,
      ShooterHandler shooterHandlerLeft,
      ShooterHandler shooterHandlerRight) {
    this.drivetrain = drivetrain;
    this.flywheelLeft = flywheelLeft;
    this.flywheelRight = flywheelRight;
    this.groundRollers = groundRollers;
    this.groundPivot = groundPivot;
    this.hoodLeft = hoodLeft;
    this.hoodRight = hoodRight;
    this.kicker = kicker;
    this.rollerFloor = rollerFloor;
    this.b2 = b2;
    this.turretLeft = turretLeft;
    this.turretRight = turretRight;
    this.shooterHandlerLeft = shooterHandlerLeft;
    this.shooterHandlerRight = shooterHandlerRight;
    applyState();
  }

  public void periodic() {
    PowerManagerState previousState = state;

    if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE
        || shooterHandlerRight.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE) {
      setState(
          shooterHandlerLeft.isShuttleTarget() || shooterHandlerRight.isShuttleTarget()
              ? PowerManagerState.SHUTTLING
              : PowerManagerState.SHOOTING);
    } else {
      setState(PowerManagerState.DEFAULT);
    }

    if (previousState != state) {
      applyState();
    }
  }

  public PowerManagerState getState() {
    return state;
  }

  public void setState(PowerManagerState newState) {
    if (state == newState) return;
    state = newState;
  }

  private void applyState() {
    Logger.recordOutput("PowerManager/State", state.name());
    Logger.recordOutput("PowerManager/FlywheelSupplyCurrent", state.flywheelSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/GroundRollersSupplyCurrent", state.groundRollersSupplyCurrent);
    Logger.recordOutput("PowerManager/GroundPivotSupplyCurrent", state.groundPivotSupplyCurrent);
    Logger.recordOutput("PowerManager/HoodSupplyCurrent", state.hoodSupplyCurrent);
    Logger.recordOutput("PowerManager/KickerSupplyCurrent", state.kickerSupplyCurrent);
    Logger.recordOutput("PowerManager/RollerFloorSupplyCurrent", state.rollerFloorSupplyCurrent);
    Logger.recordOutput("PowerManager/B2SupplyCurrent", state.b2SupplyCurrent);
    Logger.recordOutput("PowerManager/TurretSupplyCurrent", state.turretSupplyCurrent);
    Logger.recordOutput("PowerManager/DrivetrainSupplyCurrent", state.drivetrainSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/DrivetrainSteerSupplyCurrent", state.drivetrainSteerSupplyCurrent);

    drivetrain.setSupplyCurrentLimits(
        state.drivetrainSupplyCurrent, state.drivetrainSteerSupplyCurrent);
    flywheelLeft.setSupplyCurrentLimit(state.flywheelSupplyCurrent);
    flywheelRight.setSupplyCurrentLimit(state.flywheelSupplyCurrent);
    groundRollers.setSupplyCurrentLimit(state.groundRollersSupplyCurrent);
    groundPivot.setSupplyCurrentLimit(state.groundPivotSupplyCurrent);
    hoodLeft.setSupplyCurrentLimit(state.hoodSupplyCurrent);
    hoodRight.setSupplyCurrentLimit(state.hoodSupplyCurrent);
    kicker.setSupplyCurrentLimit(state.kickerSupplyCurrent);
    rollerFloor.setSupplyCurrentLimit(state.rollerFloorSupplyCurrent);
    b2.setSupplyCurrentLimit(state.b2SupplyCurrent);
    turretLeft.setSupplyCurrentLimit(state.turretSupplyCurrent);
    turretRight.setSupplyCurrentLimit(state.turretSupplyCurrent);
  }
}
