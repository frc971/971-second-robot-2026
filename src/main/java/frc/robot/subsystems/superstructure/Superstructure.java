package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.subsystems.Controllers;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

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

  private enum Goal {
    MANUAL,
    SHOOT,
    TUNER
  }

  @AutoLogOutput @Getter private Goal goal;

  public Superstructure(RobotContainer robotContainer) {
    flywheel = new Flywheel();
    hood = new Hood();
    indexer = new Indexer();
    turret = new Turret();
    groundRollers = new GroundRollers();
    shooterHandler =
        new ShooterHandler(
            turret,
            hood,
            flywheel,
            indexer,
            robotContainer.drivetrain,
            RobotBase.isReal() ? ShooterConfig.actualConfig() : ShooterConfig.testConfig());
    shooterTuner = new ShooterTuner(flywheel, hood, turret);

    setGoal(SetpointGoal.NEUTRAL);
    this.goal = Goal.TUNER;
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {
      setGoal(SetpointGoal.NEUTRAL);

      if (Controllers.XBOX.a().getAsBoolean()) setGoal(SetpointGoal.INTAKE);

      if (Controllers.XBOX.button(8).getAsBoolean()) goal = Goal.MANUAL;
      if (Controllers.XBOX.x().getAsBoolean()) goal = Goal.SHOOT;
      if (Controllers.XBOX.y().getAsBoolean()) goal = Goal.TUNER;
    }

    switch (goal) {
      case MANUAL -> {
        shooterTuner.setGoal(ShooterTuner.Goal.NONE);
        shooterHandler.setShooterGoal(ShooterHandler.Goal.NONE);

        setGoal(SetpointGoal.RESET);
      }
      case SHOOT -> {
        shooterTuner.setGoal(ShooterTuner.Goal.NONE);
        shooterHandler.setShooterGoal(ShooterHandler.Goal.SHOOT);
      }
      case TUNER -> {
        shooterTuner.setGoal(ShooterTuner.Goal.ACTIVE);
        shooterHandler.setShooterGoal(ShooterHandler.Goal.NONE);

        setGoal(SetpointGoal.INDEX);
        setGoal(SetpointGoal.INTAKE);
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
    if (setpoint.getFlywheel().isPresent()) flywheel.setVelocity(setpoint.getFlywheel().get());
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
    hood.resetPosition(SetpointGoal.RESET.getSetpoint().getHood().get());
    turret.resetPosition(SetpointGoal.RESET.getSetpoint().getTurret().get());
  }

  public Command neutral() {
    return Commands.run(() -> setGoal(SetpointGoal.NEUTRAL));
  }

  public boolean freezeDriving() {
    return goal == Goal.TUNER && shooterTuner.freezeDriving();
  }
}
