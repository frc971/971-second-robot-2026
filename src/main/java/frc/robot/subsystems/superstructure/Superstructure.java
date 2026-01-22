package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.shooter.ShooterHandler;
import frc.robot.subsystems.Controller;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

// import frc.robot.subsystems.buttonboard.ButtonBoard;

/**
 * Central place to instantiate and hold references to robot mechanism subsystems. This prevents
 * {@code RobotContainer} from being cluttered with individual mechanism construction logic and
 * makes it easier to expand later.
 */
@Getter
public class Superstructure {
  // Add subsystems here as they are created
  public final Flywheel flywheel;
  public final Hood hood;
  public final Indexer indexer;
  public final Turret turret;
  public final GroundRollers groundRollers;

  private final ShooterHandler shooterHandler;
  private final ShooterTuner shooterTuner;

  private enum State {
    IDLE,
    MANUAL,
    SHOOTING,
    TUNING
  }

  @AutoLogOutput @Getter private State state;

  public Superstructure(RobotContainer robotContainer) {
    flywheel = new Flywheel();
    hood = new Hood();
    indexer = new Indexer();
    turret = new Turret();
    groundRollers = new GroundRollers();
    shooterHandler = new ShooterHandler(robotContainer, ShooterConfig.testConfig());
    shooterTuner = new ShooterTuner(flywheel, hood, turret);

    // Default states
    setGoal(SetpointGoal.NEUTRAL);
    this.state = State.IDLE;
  }

  public void periodic() {

    if (DriverStation.isTeleop()) {
      setGoal(SetpointGoal.NEUTRAL);

      if (Controller.GROUND_ROLLERS.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE);
      }

      // TODO: check for switching states
      switch (state) {
        case IDLE -> {
          if (Controller.SHOOT_BUTTON.getAsBoolean()) {
            state = State.SHOOTING;
          }
        }

        case MANUAL -> {
          shooterTuner.setGoal(ShooterTuner.Goal.NONE);
          // TODO: add setpoints for Troy
        }

        case SHOOTING -> {
          shooterTuner.setGoal(ShooterTuner.Goal.NONE);
          if (Controller.SHOOT_BUTTON.getAsBoolean() || !shooterHandler.satisfiesConstraints()) {
            state = State.IDLE;
          }
          if (shooterHandler.ready()) {
            setGoal(SetpointGoal.INDEX);
          }

          flywheel.setVelocity(shooterHandler.getFlywheelAngularVelocity());
          hood.setPosition(shooterHandler.getLaunchSolution().hoodAngle());
          turret.setPosition(shooterHandler.getRelativeTurretAngle());
        }

        case TUNING -> {
          shooterTuner.setGoal(ShooterTuner.Goal.ACTIVE);
        }
        default -> {}
      }
    }

    shooterHandler.periodic();
    shooterTuner.periodic();

    // subsystems
    flywheel.periodic();
    hood.periodic();
    indexer.periodic();
    turret.periodic();
    groundRollers.periodic();
  }

  public void setGoal(Setpoint setpoint) {
    if (setpoint.getFlywheel().isPresent()) flywheel.setPosition(setpoint.getFlywheel().get());
    if (setpoint.getHood().isPresent()) hood.setPosition(setpoint.getHood().get());
    if (setpoint.getTurret().isPresent()) turret.setPosition(setpoint.getTurret().get());
    if (setpoint.getIndexer().isPresent()) indexer.setVoltage(setpoint.getIndexer().get());
    if (setpoint.getGroundRollers().isPresent())
      groundRollers.setVoltage(setpoint.getGroundRollers().get());
  }

  public void setGoal(SetpointGoal setpoint) {
    setGoal(setpoint.getSetpoint());
  }

  public void resetPositions() {
    flywheel.resetPosition(SetpointGoal.RESET.getSetpoint().getFlywheel().get());
    hood.resetPosition(SetpointGoal.RESET.getSetpoint().getHood().get());
    turret.resetPosition(SetpointGoal.RESET.getSetpoint().getTurret().get());
  }

  public Command neutral() {
    return Commands.run(() -> setGoal(SetpointGoal.NEUTRAL));
  }
}
