package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.power.BatteryEstimator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class PowerManager {

    private enum PowerManagerState {
        SHOOTING,
        SHUTTLING,
        SUPERCHARGED,
        DEFAULT,
    }
  private static final int UPDATE_PERIOD_LOOPS = 10;
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double UPDATE_PERIOD_SECONDS = UPDATE_PERIOD_LOOPS * LOOP_PERIOD_SECONDS;

  private static final double LOOKAHEAD_SECONDS = 0.20;

  private static final double MAX_CURRENT_SLOPE = 500.0; 
  private static final double MIN_BATTERY_VOLTAGE = 7.5;

  private static final double LIMIT_SLEW_RATE = 150.0;

  private static final int DRIVETRAIN_DRIVE_MOTORS = 4;
  private static final int DRIVETRAIN_STEER_MOTORS = 4;
  private static final int FLYWHEEL_MOTORS = 2;
  private static final int TURRET_MOTORS = 2;

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

  private final BatteryEstimator batteryEstimator = new BatteryEstimator();

  private int loopCounter = 0;

  private double previousCurrent = 0.0;
  private double currentSlope = 0.0;
  private double predictedCurrent = 0.0;

  private double drivetrainCurrentLimit = 0.0;
  private double drivetrainSteerCurrentLimit = 0.0;

  private double flywheelCurrentLimit = 0.0;
  private double groundRollersCurrentLimit = 0.0;
  private double groundPivotCurrentLimit = 0.0;
  private double hoodCurrentLimit = 0.0;
  private double kickerCurrentLimit = 0.0;
  private double rollerFloorCurrentLimit = 0.0;
  private double b2CurrentLimit = 0.0;
  private double turretCurrentLimit = 0.0;

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

    applyLimits();
  }

  public void periodic() {
    updateState();

    loopCounter++;

    if (loopCounter >= UPDATE_PERIOD_LOOPS) {
      loopCounter = 0;

      updatePowerModel();
      calculateLimits();
      applyLimits();
    }
  }

  private void updateState() {
    PowerManagerState previousState = state;

    if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE
        || shooterHandlerRight.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE) {

      if (shooterHandlerLeft.isShuttleTarget() || shooterHandlerRight.isShuttleTarget()) {

        setState(PowerManagerState.SHUTTLING);

      } else {

        setState(PowerManagerState.SHOOTING);
      }

    } else {
      setState(PowerManagerState.DEFAULT);
    }

    if (previousState != state) {
      Logger.recordOutput("PowerManager/State", state.name());
    }
  }

  private void updatePowerModel() {
    double current = Math.max(0.0, RobotController.getInputCurrent());
    double voltage = RobotController.getBatteryVoltage();

    currentSlope =
        MathUtil.clamp(
            (current - previousCurrent) / UPDATE_PERIOD_SECONDS,
            -MAX_CURRENT_SLOPE,
            MAX_CURRENT_SLOPE);

    previousCurrent = current;

    predictedCurrent = Math.max(current, current + currentSlope * LOOKAHEAD_SECONDS);

    batteryEstimator.update(current, voltage);

    Logger.recordOutput("PowerManager/ActualCurrent", current);
    Logger.recordOutput("PowerManager/CurrentSlope", currentSlope);
    Logger.recordOutput("PowerManager/PredictedCurrent", predictedCurrent);
    Logger.recordOutput("PowerManager/BatteryVoltage", voltage);
    Logger.recordOutput(
        "PowerManager/BatteryCurrentLimit",
        batteryEstimator.calculateMaxCurrent(MIN_BATTERY_VOLTAGE));
  }

  private void calculateLimits() {

    double batteryLimit = batteryEstimator.calculateMaxCurrent(MIN_BATTERY_VOLTAGE);

    double lookaheadMargin = predictedCurrent - RobotController.getInputCurrent();

    double availableCurrent = Math.max(0.0, batteryLimit - Math.max(0.0, lookaheadMargin));

    availableCurrent = Math.min(availableCurrent, predictedCurrent + 80.0);

    double drivetrainBudget = availableCurrent * allocation.drivetrain;

    double flywheelBudget = availableCurrent * allocation.flywheel;

    double groundRollersBudget = availableCurrent * allocation.groundRollers;

    double groundPivotBudget = availableCurrent * allocation.groundPivot;

    double hoodBudget = availableCurrent * allocation.hood;

    double kickerBudget = availableCurrent * allocation.kicker;

    double rollerFloorBudget = availableCurrent * allocation.rollerFloor;

    double b2Budget = availableCurrent * allocation.b2;

    double turretBudget = availableCurrent * allocation.turret;

    double targetDrivetrainLimit = drivetrainBudget * 0.80 / DRIVETRAIN_DRIVE_MOTORS;

    double targetDrivetrainSteerLimit = drivetrainBudget * 0.20 / DRIVETRAIN_STEER_MOTORS;

    double targetFlywheelLimit = flywheelBudget / FLYWHEEL_MOTORS;

    double targetTurretLimit = turretBudget / TURRET_MOTORS;

    double targetGroundRollersLimit = groundRollersBudget;
    double targetGroundPivotLimit = groundPivotBudget;
    double targetHoodLimit = hoodBudget;
    double targetKickerLimit = kickerBudget;
    double targetRollerFloorLimit = rollerFloorBudget;
    double targetB2Limit = b2Budget;

    drivetrainCurrentLimit = slew(drivetrainCurrentLimit, targetDrivetrainLimit);

    drivetrainSteerCurrentLimit = slew(drivetrainSteerCurrentLimit, targetDrivetrainSteerLimit);

    flywheelCurrentLimit = slew(flywheelCurrentLimit, targetFlywheelLimit);

    groundRollersCurrentLimit = slew(groundRollersCurrentLimit, targetGroundRollersLimit);

    groundPivotCurrentLimit = slew(groundPivotCurrentLimit, targetGroundPivotLimit);

    hoodCurrentLimit = slew(hoodCurrentLimit, targetHoodLimit);

    kickerCurrentLimit = slew(kickerCurrentLimit, targetKickerLimit);

    rollerFloorCurrentLimit = slew(rollerFloorCurrentLimit, targetRollerFloorLimit);

    b2CurrentLimit = slew(b2CurrentLimit, targetB2Limit);

    turretCurrentLimit = slew(turretCurrentLimit, targetTurretLimit);

    logLimits(availableCurrent);
  }

  private void applyLimits() {

    drivetrain.setSupplyCurrentLimits(drivetrainCurrentLimit, drivetrainSteerCurrentLimit);

    flywheelLeft.setSupplyCurrentLimit(flywheelCurrentLimit);
    flywheelRight.setSupplyCurrentLimit(flywheelCurrentLimit);

    groundRollers.setSupplyCurrentLimit(groundRollersCurrentLimit);
    groundPivot.setSupplyCurrentLimit(groundPivotCurrentLimit);

    hoodLeft.setSupplyCurrentLimit(hoodCurrentLimit);
    hoodRight.setSupplyCurrentLimit(hoodCurrentLimit);

    kicker.setSupplyCurrentLimit(kickerCurrentLimit);
    rollerFloor.setSupplyCurrentLimit(rollerFloorCurrentLimit);

    b2.setSupplyCurrentLimit(b2CurrentLimit);

    turretLeft.setSupplyCurrentLimit(turretCurrentLimit);
    turretRight.setSupplyCurrentLimit(turretCurrentLimit);
  }

  private double slew(double current, double target) {
    double maxChange = LIMIT_SLEW_RATE * UPDATE_PERIOD_SECONDS;

    return MathUtil.clamp(target, current - maxChange, current + maxChange);
  }

  private void logLimits(double availableCurrent) {

    Logger.recordOutput("PowerManager/AvailableCurrent", availableCurrent);

    Logger.recordOutput("PowerManager/DrivetrainCurrentLimit", drivetrainCurrentLimit);

    Logger.recordOutput("PowerManager/DrivetrainSteerCurrentLimit", drivetrainSteerCurrentLimit);

    Logger.recordOutput("PowerManager/FlywheelCurrentLimit", flywheelCurrentLimit);

    Logger.recordOutput("PowerManager/GroundRollersCurrentLimit", groundRollersCurrentLimit);

    Logger.recordOutput("PowerManager/GroundPivotCurrentLimit", groundPivotCurrentLimit);

    Logger.recordOutput("PowerManager/HoodCurrentLimit", hoodCurrentLimit);

    Logger.recordOutput("PowerManager/KickerCurrentLimit", kickerCurrentLimit);

    Logger.recordOutput("PowerManager/RollerFloorCurrentLimit", rollerFloorCurrentLimit);

    Logger.recordOutput("PowerManager/B2CurrentLimit", b2CurrentLimit);

    Logger.recordOutput("PowerManager/TurretCurrentLimit", turretCurrentLimit);
  }
}
