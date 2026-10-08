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
import java.util.function.BooleanSupplier;
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
  private final BooleanSupplier manualMode;

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
      BooleanSupplier manualMode) {
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
    this.manualMode = manualMode;
    applyState();
  }

  public void periodic() {
    if (manualMode.getAsBoolean()) {
      state = PowerManagerState.MANUAL;
    } else if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE
        || shooterHandlerRight.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE) {
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

    drivetrain.setSupplyCurrentLimits(state.drivetrain(), state.drivetrainSteer());
    flywheelLeft.setSupplyCurrentLimit(state.flywheel());
    flywheelRight.setSupplyCurrentLimit(state.flywheel());
    groundRollers.setSupplyCurrentLimit(state.groundRollers());
    groundPivot.setSupplyCurrentLimit(state.groundPivot());
    hoodLeft.setSupplyCurrentLimit(state.hood());
    hoodRight.setSupplyCurrentLimit(state.hood());
    kicker.setSupplyCurrentLimit(state.kicker());
    rollerFloor.setSupplyCurrentLimit(state.rollerFloor());
    b2.setSupplyCurrentLimit(state.b2());
    turretLeft.setSupplyCurrentLimit(state.turret());
    turretRight.setSupplyCurrentLimit(state.turret());
    lastAppliedState = state;
  }
}
