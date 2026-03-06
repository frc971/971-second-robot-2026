package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.superstructure.AngularSubsystem;
import frc.robot.subsystems.Controllers;
import java.util.Stack;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

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

  public enum Mode {
    NONE,
    DRIVING,
    HOOD,
    FLYWHEEL
  }

  public enum ShotResult {
    OVERSHOOT,
    UNDERSHOOT,
    HIT
  }

  public enum Arc {
    LOW,
    HIGH
  }

  private record TunerState(
      AngularVelocity flywheelSpeed, Angle hoodAngle, double minVal, double maxVal) {}

  private AngularSubsystem flywheel;
  private AngularSubsystem hood;
  private AngularSubsystem turret;
  private final ShooterHandler shooterHandler;
  private final ShooterConfig config;

  @AutoLogOutput @Setter @Getter private Goal goal = Goal.NONE;
  @AutoLogOutput @Getter private Mode mode = Mode.DRIVING;
  @AutoLogOutput @Getter private ShotResult lastShotResult = ShotResult.HIT;
  @AutoLogOutput @Getter private boolean indexing = false;

  @AutoLogOutput private AngularVelocity flywheelSpeed = RotationsPerSecond.of(RESET_FLYWHEEL_RPS);
  @AutoLogOutput private Angle hoodAngle = Degrees.of(RESET_HOOD_DEGREES);
  @AutoLogOutput @Setter private double currentDistance = 0;

  private static final double MIN_FLYWHEEL_RPS = 0.0, MAX_FLYWHEEL_RPS = 100.0;
  private static final double MIN_HOOD_DEGREES = 0, MAX_HOOD_DEGREES = 50.0;
  private static final double RESET_FLYWHEEL_RPS = 50, RESET_HOOD_DEGREES = 25;

  @AutoLogOutput private double flywheelMinValue = MIN_FLYWHEEL_RPS;
  @AutoLogOutput private double flywheelMaxValue = MAX_FLYWHEEL_RPS;
  @AutoLogOutput private double hoodMinValue = MIN_HOOD_DEGREES;
  @AutoLogOutput private double hoodMaxValue = MAX_HOOD_DEGREES;

  private final Stack<TunerState> flywheelHistory = new Stack<>();
  private final Stack<TunerState> hoodHistory = new Stack<>();

  public ShooterTuner(
      AngularSubsystem flywheel,
      AngularSubsystem hood,
      AngularSubsystem turret,
      ShooterHandler shooterHandler) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.shooterHandler = shooterHandler;
    this.config = shooterHandler.getConfig();
  }

  public void periodic() {
    if (goal == Goal.NONE) {
      mode = Mode.NONE;
      return;
    }

    if (Controllers.TOGGLE_HOOD_FLYWHEEL.rising()) {
      mode = (mode == Mode.DRIVING || mode == Mode.HOOD) ? Mode.FLYWHEEL : Mode.HOOD;
    }

    if (Controllers.REVERT.rising()) revertToPrevious();
    if (Controllers.UNDERSHOOT.rising()) applyShotResult(ShotResult.UNDERSHOOT);
    if (Controllers.OVERSHOOT.rising()) applyShotResult(ShotResult.OVERSHOOT);
    if (Controllers.HIT.rising()) applyShotResult(ShotResult.HIT);

    indexing = Controllers.INDEX.getAsBoolean();

    flywheel.setVelocity(flywheelSpeed);
    hood.setPosition(hoodAngle);
    turret.setPosition(shooterHandler.getRelativeTurretAngle());
  }

  private void applyShotResult(ShotResult result) {
    lastShotResult = result;
    if (result == ShotResult.HIT) {
      config.PHYSICS().SHOT_TABLE().put(shooterHandler.currentDistance(), hoodAngle, flywheelSpeed);
      reset();
      mode = Mode.DRIVING;
    } else {
      double currentValue =
          (mode == Mode.FLYWHEEL) ? flywheelSpeed.in(RotationsPerSecond) : hoodAngle.in(Degrees);

      if (mode == Mode.FLYWHEEL) {
        if (result == ShotResult.OVERSHOOT) flywheelMaxValue = currentValue;
        else if (result == ShotResult.UNDERSHOOT) flywheelMinValue = currentValue;

        double newValue = (flywheelMinValue + flywheelMaxValue) / 2;
        flywheelSpeed =
            RotationsPerSecond.of(MathUtil.clamp(newValue, MIN_FLYWHEEL_RPS, MAX_FLYWHEEL_RPS));
        flywheelHistory.push(
            new TunerState(flywheelSpeed, hoodAngle, flywheelMinValue, flywheelMaxValue));
      } else if (mode == Mode.HOOD) {
        if (result == ShotResult.OVERSHOOT) hoodMaxValue = currentValue;
        else if (result == ShotResult.UNDERSHOOT) hoodMinValue = currentValue;

        double newValue = (hoodMinValue + hoodMaxValue) / 2;
        hoodAngle = Degrees.of(MathUtil.clamp(newValue, MIN_HOOD_DEGREES, MAX_HOOD_DEGREES));
        hoodHistory.push(new TunerState(flywheelSpeed, hoodAngle, hoodMinValue, hoodMaxValue));
      }
    }
  }

  private void revertToPrevious() {
    Stack<TunerState> history = (mode == Mode.FLYWHEEL) ? flywheelHistory : hoodHistory;
    if (history.isEmpty()) {
      return;
    }
    TunerState previous = history.pop();
    if (mode == Mode.FLYWHEEL) {
      flywheelSpeed = previous.flywheelSpeed;
      flywheelMinValue = previous.minVal;
      flywheelMaxValue = previous.maxVal;
    } else if (mode == Mode.HOOD) {
      hoodAngle = previous.hoodAngle;
      hoodMinValue = previous.minVal;
      hoodMaxValue = previous.maxVal;
    }
  }

  private void reset() {
    System.out.println("SHOT TABLE ENTRIES");
    System.out.println(config.PHYSICS().SHOT_TABLE().printSingleLine());

    flywheelSpeed = RotationsPerSecond.of(RESET_FLYWHEEL_RPS);
    hoodAngle = Degrees.of(RESET_HOOD_DEGREES);

    flywheelMinValue = MIN_FLYWHEEL_RPS;
    flywheelMaxValue = MAX_FLYWHEEL_RPS;
    hoodMinValue = MIN_HOOD_DEGREES;
    hoodMaxValue = MAX_HOOD_DEGREES;
  }
}
