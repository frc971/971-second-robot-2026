package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.power.BatteryEstimator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class PowerManager {
  private static final int UPDATE_PERIOD_LOOPS = 10;
  private static final double UPDATE_SECONDS = UPDATE_PERIOD_LOOPS * 0.02;
  private static final double LOOKAHEAD_SECONDS = 0.10;
  private static final double MIN_VOLTAGE = 7.5;
  private static final double MAX_SLOPE = 500.0;
  private static final double RECOVERY_AMPS_PER_SECOND = 150.0;
  private static final double FALLBACK_BUDGET = 120.0;
  private static final double MAX_BUDGET = 300.0;

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
  private final BatteryEstimator estimator = new BatteryEstimator();

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter steerLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter flywheelLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter rollersLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter pivotLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter hoodLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter kickerLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter floorLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter b2Limiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);
  private final SlewRateLimiter turretLimiter = new SlewRateLimiter(RECOVERY_AMPS_PER_SECOND);

  private PowerManagerState state = PowerManagerState.DEFAULT;
  private boolean sampled;
  private int loops;
  private double previousCurrent;

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
  }

  public PowerManagerState getState() {
    return state;
  }

  public void periodic() {
    updateState();
    if (++loops < UPDATE_PERIOD_LOOPS) return;
    loops = 0;
    updatePowerModel();
  }

  private void updateState() {
    boolean left = shooterHandlerLeft.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE;
    boolean right = shooterHandlerRight.getShooterGoal() == ShooterHandler.ShooterGoal.ACTIVE;
    PowerManagerState next =
        !(left || right)
            ? PowerManagerState.DEFAULT
            : (left && shooterHandlerLeft.isShuttleTarget())
                    || (right && shooterHandlerRight.isShuttleTarget())
                ? PowerManagerState.SHUTTLING
                : PowerManagerState.SHOOTING;
    
    if (next != state) {
      state = next;
      Logger.recordOutput("PowerManager/State", state.name());
    }
  }

  private void updatePowerModel() {
    double current = RobotController.getInputCurrent();
    double voltage = RobotController.getBatteryVoltage();
    if (!Double.isFinite(current) || current < 0 || !Double.isFinite(voltage) || voltage <= 0) {
      applyLimits(FALLBACK_BUDGET);
      return;
    }

    if (!sampled) {
      previousCurrent = current;
      sampled = true;
    }

    double slope =
        MathUtil.clamp((current - previousCurrent) / UPDATE_SECONDS, -MAX_SLOPE, MAX_SLOPE);
    previousCurrent = current;
    double predicted = Math.max(current, current + slope * LOOKAHEAD_SECONDS);
    estimator.update(current, voltage);
    double batteryLimit = estimator.calculateMaxCurrent(MIN_VOLTAGE);
    double budget =
        Double.isFinite(batteryLimit) && batteryLimit > 0
            ? MathUtil.clamp(batteryLimit - Math.max(0, predicted - current), 0, MAX_BUDGET)
            : FALLBACK_BUDGET;
    
    Logger.recordOutput("PowerManager/ActualCurrent", current);
    Logger.recordOutput("PowerManager/PredictedCurrent", predicted);
    Logger.recordOutput("PowerManager/BatteryVoltage", voltage);
    Logger.recordOutput("PowerManager/BatteryCurrentLimit", batteryLimit);
    Logger.recordOutput("PowerManager/Budget", budget);

    applyLimits(budget);
  }

  private static double recover(SlewRateLimiter limiter, double target) {
    target = Math.max(0, target);
    if (target < limiter.lastValue()) limiter.reset(target);
    return limiter.calculate(target);
  }

  private void applyLimits(double budget) {
    PowerManagerState.Allocation a = state.allocation();
    double drive = recover(driveLimiter, budget * a.drivetrain() * .80 / 4);
    double steer = recover(steerLimiter, budget * a.drivetrain() * .20 / 4);
    double flywheel = recover(flywheelLimiter, budget * a.flywheel() / 2);
    double rollers = recover(rollersLimiter, budget * a.groundRollers());
    double pivot = recover(pivotLimiter, budget * a.groundPivot());
    double hood = recover(hoodLimiter, budget * a.hood() / 2);
    double kickerLimit = recover(kickerLimiter, budget * a.kicker());
    double floor = recover(floorLimiter, budget * a.rollerFloor());
    double b2Limit = recover(b2Limiter, budget * a.b2());
    double turret = recover(turretLimiter, budget * a.turret() / 2);

    drivetrain.setSupplyCurrentLimits(drive, steer);
    flywheelLeft.setSupplyCurrentLimit(flywheel);
    flywheelRight.setSupplyCurrentLimit(flywheel);
    groundRollers.setSupplyCurrentLimit(rollers);
    groundPivot.setSupplyCurrentLimit(pivot);
    hoodLeft.setSupplyCurrentLimit(hood);
    hoodRight.setSupplyCurrentLimit(hood);
    kicker.setSupplyCurrentLimit(kickerLimit);
    rollerFloor.setSupplyCurrentLimit(floor);
    b2.setSupplyCurrentLimit(b2Limit);
    turretLeft.setSupplyCurrentLimit(turret);
    turretRight.setSupplyCurrentLimit(turret);
    
    Logger.recordOutput("PowerManager/DriveLimit", drive);
    Logger.recordOutput("PowerManager/SteerLimit", steer);
    Logger.recordOutput("PowerManager/FlywheelLimit", flywheel);
    Logger.recordOutput("PowerManager/HoodLimit", hood);
    Logger.recordOutput("PowerManager/TurretLimit", turret);
  }
}
