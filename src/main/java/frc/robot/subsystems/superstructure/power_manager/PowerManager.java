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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.TurretLeft;
import frc.robot.subsystems.superstructure.TurretRight;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
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
  private final Superstructure superstructure;
  private final ExecutorService limitExecutor = Executors.newFixedThreadPool(2);

  @Getter
  @AutoLogOutput(key = "PowerManager/State")
  private PowerManagerState state = PowerManagerState.DEFAULT;

  private PowerManagerState lastAppliedState;

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
      ShooterHandler shooterHandlerRight,
      Superstructure superstructure) {
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
    this.superstructure = superstructure;
    applyState();
  }

  public void periodic() {
    if (superstructure.getMode() == Superstructure.Mode.MANUAL) {
      state = PowerManagerState.MANUAL;
    } else if (superstructure.isWantsShot()
        && (shooterHandlerLeft.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE
            || shooterHandlerRight.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE)) {
      state =
          shooterHandlerLeft.isShuttleTarget() || shooterHandlerRight.isShuttleTarget()
              ? PowerManagerState.SHUTTLING
              : PowerManagerState.SHOOTING;
    } else {
      state = PowerManagerState.DEFAULT;
    }

    if (lastAppliedState != state) {
      applyState();
    }
  }

  private void applyState() {
    PowerManagerState stateToApply = state;
    Logger.recordOutput("PowerManager/FlywheelSupplyCurrent", state.flywheel());
    Logger.recordOutput("PowerManager/GroundRollersSupplyCurrent", state.groundRollers());
    Logger.recordOutput("PowerManager/GroundPivotSupplyCurrent", state.groundPivot());
    Logger.recordOutput("PowerManager/HoodSupplyCurrent", state.hood());
    Logger.recordOutput("PowerManager/KickerSupplyCurrent", state.kicker());
    Logger.recordOutput("PowerManager/RollerFloorSupplyCurrent", state.rollerFloor());
    Logger.recordOutput("PowerManager/B2SupplyCurrent", state.b2());
    Logger.recordOutput("PowerManager/TurretSupplyCurrent", state.turret());
    Logger.recordOutput("PowerManager/DrivetrainSupplyCurrent", state.drivetrain());
    Logger.recordOutput("PowerManager/DrivetrainSteerSupplyCurrent", state.drivetrainSteer());
    limitExecutor.submit(() -> applyLimits(stateToApply));
  }

  private void applyLimits(PowerManagerState state) {
    CompletableFuture<?>[] limitTasks = {
      CompletableFuture.runAsync(
          () -> drivetrain.setSupplyCurrentLimits(state.drivetrain(), state.drivetrainSteer()),
          limitExecutor),
      CompletableFuture.runAsync(
          () -> flywheelLeft.setSupplyCurrentLimit(state.flywheel()), limitExecutor),
      CompletableFuture.runAsync(
          () -> flywheelRight.setSupplyCurrentLimit(state.flywheel()), limitExecutor),
      CompletableFuture.runAsync(
          () -> groundRollers.setSupplyCurrentLimit(state.groundRollers()), limitExecutor),
      CompletableFuture.runAsync(
          () -> groundPivot.setSupplyCurrentLimit(state.groundPivot()), limitExecutor),
      CompletableFuture.runAsync(() -> hoodLeft.setSupplyCurrentLimit(state.hood()), limitExecutor),
      CompletableFuture.runAsync(
          () -> hoodRight.setSupplyCurrentLimit(state.hood()), limitExecutor),
      CompletableFuture.runAsync(() -> kicker.setSupplyCurrentLimit(state.kicker()), limitExecutor),
      CompletableFuture.runAsync(
          () -> rollerFloor.setSupplyCurrentLimit(state.rollerFloor()), limitExecutor),
      CompletableFuture.runAsync(() -> b2.setSupplyCurrentLimit(state.b2()), limitExecutor),
      CompletableFuture.runAsync(
          () -> turretLeft.setSupplyCurrentLimit(state.turret()), limitExecutor),
      CompletableFuture.runAsync(
          () -> turretRight.setSupplyCurrentLimit(state.turret()), limitExecutor)
    };

    CompletableFuture.allOf(limitTasks).thenRun(() -> lastAppliedState = state);
  }
}
