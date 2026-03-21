package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.superstructure.AngularSubsystem;
import frc.robot.subsystems.Controllers;
import java.util.Stack;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*
 * ShotTuner class to collect data for the shot table. Handles shooting parameters
 * based on shot feedback (overshoot/undershoot/hit), and uses binary search to find
 * optimal values.
 */
public class ShooterTuner {

  public enum Goal {
    ACTIVE,
    NONE
  }

  public enum ShotResult {
    OVERSHOOT,
    UNDERSHOOT,
    HIT
  }

  private record TunerState(AngularVelocity flywheelSpeed, double minVal, double maxVal) {}

  private AngularSubsystem flywheel;
  private Hood hood;
  private AngularSubsystem turret;
  private final ShooterHandler shooterHandler;
  private final ShooterConfig config;

  @AutoLogOutput @Setter @Getter private Goal goal = Goal.NONE;
  @AutoLogOutput @Getter private ShotResult lastShotResult = ShotResult.HIT;
  @AutoLogOutput @Getter private boolean indexing = false;

  @AutoLogOutput private AngularVelocity flywheelSpeed = RotationsPerSecond.of(RESET_FLYWHEEL_RPS);
  @AutoLogOutput private LinearVelocity requiredExitSpeed = MetersPerSecond.of(0);

  private static final double MIN_FLYWHEEL_RPS = 0.0, MAX_FLYWHEEL_RPS = 100.0;
  private static final double RESET_FLYWHEEL_RPS = 50;

  @AutoLogOutput private double flywheelMinValue = MIN_FLYWHEEL_RPS;
  @AutoLogOutput private double flywheelMaxValue = MAX_FLYWHEEL_RPS;

  private final Stack<TunerState> flywheelHistory = new Stack<>();

  public ShooterTuner(
      AngularSubsystem flywheel,
      Hood hood,
      AngularSubsystem turret,
      ShooterHandler shooterHandler) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.shooterHandler = shooterHandler;
    this.config = shooterHandler.getConfig();
  }

  public void periodic() {
    if (goal == Goal.NONE) return;

    if (Controllers.REVERT.rising()) revertToPrevious();
    if (Controllers.UNDERSHOOT.rising()) applyShotResult(ShotResult.UNDERSHOOT);
    if (Controllers.OVERSHOOT.rising()) applyShotResult(ShotResult.OVERSHOOT);
    if (Controllers.HIT.rising()) applyShotResult(ShotResult.HIT);

    indexing = Controllers.INDEX.getAsBoolean();

    LaunchSolution solution = shooterHandler.getLaunchSolution();
    if (solution != null) {
      requiredExitSpeed = computeExitSpeed();

      Angle solverHood = solution.hoodAngle();
      Angle solverTurret = shooterHandler.getDirectRelativeTranslation().getAngle().getMeasure();
      hood.setPosition(solverHood);
      turret.setPosition(solverTurret);
    }

    flywheel.setVelocity(flywheelSpeed);
  }

  private LinearVelocity computeExitSpeed() {
    ObjectState proj = shooterHandler.getProjectileState();
    ObjectState target = shooterHandler.getTargetState();
    double distance = target.minus(proj).xyPos().getNorm();
    Logger.recordOutput("ShooterTuner/Distance", distance);

    if (proj == null || target == null) return requiredExitSpeed;
    double targetAngle = config.PHYSICS().VELOCITY_ANGLE_AT_TARGET().in(Radians);
    double shotAngle =
        Math.atan(
            2 * (target.position().getZ() - proj.position().getZ()) / distance
                - Math.tan(targetAngle));

    double tanDiff = Math.tan(targetAngle) - Math.tan(shotAngle);
    double exitSpeed =
        (1.0 / Math.cos(shotAngle))
            * Math.sqrt((config.PHYSICS().GRAVITY() * distance) / Math.abs(tanDiff));
    return MetersPerSecond.of(exitSpeed);
  }

  private void applyShotResult(ShotResult result) {
    lastShotResult = result;
    if (result == ShotResult.HIT) {
      config.PHYSICS().EXIT_SPEED_TABLE().put(requiredExitSpeed, flywheelSpeed);
      reset();
    } else {
      double currentValue = flywheelSpeed.in(RotationsPerSecond);

      if (result == ShotResult.OVERSHOOT) flywheelMaxValue = currentValue;
      else if (result == ShotResult.UNDERSHOOT) flywheelMinValue = currentValue;

      double newValue = (flywheelMinValue + flywheelMaxValue) / 2;
      flywheelSpeed =
          RotationsPerSecond.of(MathUtil.clamp(newValue, MIN_FLYWHEEL_RPS, MAX_FLYWHEEL_RPS));
      flywheelHistory.push(new TunerState(flywheelSpeed, flywheelMinValue, flywheelMaxValue));
    }
  }

  private void revertToPrevious() {
    Stack<TunerState> history = flywheelHistory;
    if (history.isEmpty()) {
      return;
    }
    TunerState previous = history.pop();
    flywheelSpeed = previous.flywheelSpeed;
    flywheelMinValue = previous.minVal;
    flywheelMaxValue = previous.maxVal;
  }

  private void reset() {
    System.out.println("SHOT TABLE ENTRIES");
    System.out.println(config.PHYSICS().EXIT_SPEED_TABLE().printSingleLine());

    flywheelSpeed = RotationsPerSecond.of(RESET_FLYWHEEL_RPS);
    flywheelMinValue = MIN_FLYWHEEL_RPS;
    flywheelMaxValue = MAX_FLYWHEEL_RPS;
  }
}
