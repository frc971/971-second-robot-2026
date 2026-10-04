package frc.robot.subsystems.superstructure.power_manager;

import frc.robot.lib.superstructure.MotorSubsystem;
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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.littletonrobotics.junction.Logger;

public class PowerManager {
  private final CommandSwerveDrivetrain drivetrain;
  private final MotorSubsystem flywheelLeft;
  private final MotorSubsystem flywheelRight;
  private final MotorSubsystem groundRollers;
  private final MotorSubsystem groundPivot;
  private final MotorSubsystem hoodLeft;
  private final MotorSubsystem hoodRight;
  private final MotorSubsystem kicker;
  private final MotorSubsystem rollerFloor;
  private final MotorSubsystem b2;
  private final MotorSubsystem turretLeft;
  private final MotorSubsystem turretRight;
  private final ShooterHandler shooterHandlerLeft;
  private final ShooterHandler shooterHandlerRight;
  private final ExecutorService executor = Executors.newSingleThreadExecutor();

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

    if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.Goal.ACTIVE
        || shooterHandlerRight.getShooterGoal() == ShooterHandler.Goal.ACTIVE) {
      setState(
          shooterHandlerLeft.isShuttleTarget() || shooterHandlerRight.isShuttleTarget()
              ? PowerManagerState.SHUTTLING
              : PowerManagerState.SUPERCHARGED);
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
    PowerManagerState requestedState = state;
    executor.submit(() -> applyState(requestedState));
  }

  private void applyState(PowerManagerState requestedState) {
    Logger.recordOutput("PowerManager/State", requestedState.name());
    Logger.recordOutput("PowerManager/FlywheelSupplyCurrent", requestedState.flywheelSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/GroundRollersSupplyCurrent", requestedState.groundRollersSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/GroundPivotSupplyCurrent", requestedState.groundPivotSupplyCurrent);
    Logger.recordOutput("PowerManager/HoodSupplyCurrent", requestedState.hoodSupplyCurrent);
    Logger.recordOutput("PowerManager/KickerSupplyCurrent", requestedState.kickerSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/RollerFloorSupplyCurrent", requestedState.rollerFloorSupplyCurrent);
    Logger.recordOutput("PowerManager/B2SupplyCurrent", requestedState.b2SupplyCurrent);
    Logger.recordOutput("PowerManager/TurretSupplyCurrent", requestedState.turretSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/DrivetrainSupplyCurrent", requestedState.drivetrainSupplyCurrent);
    Logger.recordOutput(
        "PowerManager/DrivetrainSteerSupplyCurrent", requestedState.drivetrainSteerSupplyCurrent);

    drivetrain.setSupplyCurrentLimits(
        requestedState.drivetrainSupplyCurrent, requestedState.drivetrainSteerSupplyCurrent);
    flywheelLeft.setSupplyCurrentLimit(requestedState.flywheelSupplyCurrent);
    flywheelRight.setSupplyCurrentLimit(requestedState.flywheelSupplyCurrent);
    groundRollers.setSupplyCurrentLimit(requestedState.groundRollersSupplyCurrent);
    groundPivot.setSupplyCurrentLimit(requestedState.groundPivotSupplyCurrent);
    hoodLeft.setSupplyCurrentLimit(requestedState.hoodSupplyCurrent);
    hoodRight.setSupplyCurrentLimit(requestedState.hoodSupplyCurrent);
    kicker.setSupplyCurrentLimit(requestedState.kickerSupplyCurrent);
    rollerFloor.setSupplyCurrentLimit(requestedState.rollerFloorSupplyCurrent);
    b2.setSupplyCurrentLimit(requestedState.b2SupplyCurrent);
    turretLeft.setSupplyCurrentLimit(requestedState.turretSupplyCurrent);
    turretRight.setSupplyCurrentLimit(requestedState.turretSupplyCurrent);
  }
}
