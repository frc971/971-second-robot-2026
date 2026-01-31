package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.superstructure.Setpoint.Side;
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

  public final FlywheelRight flywheelRight;
  public final FlywheelLeft flywheelLeft;

  public final HoodRight hoodRight;
  public final HoodLeft hoodLeft;

  public final SpindexerLeft indexerLeft;
  public final SpindexerRight indexerRight;
  public final GroundRollers groundRollers;
  public final GroundPivot groundPivot;

  public final ShooterTuner shooterTunerRight;
  public final ShooterTuner shooterTunerLeft;

  public final ShooterHandler shooterHandlerRight;
  public final ShooterHandler shooterHandlerLeft;

  public final TurretRight turretRight;
  public final TurretLeft turretLeft;

  public final Kicker kicker;

  private enum Goal {
    MANUAL,
    SHOOT,
    TUNER
  }

  @AutoLogOutput @Getter private Goal goal;

  public Superstructure(RobotContainer robotContainer) {
    flywheelRight = new FlywheelRight();
    flywheelLeft = new FlywheelLeft();
    hoodRight = new HoodRight();
    hoodLeft = new HoodLeft();
    indexerLeft = new SpindexerLeft();
    indexerRight = new SpindexerRight();
    turretRight = new TurretRight();
    turretLeft = new TurretLeft();
    groundPivot = new GroundPivot();
    groundRollers = new GroundRollers();
    kicker = new Kicker();

    shooterTunerRight = new ShooterTuner(flywheelRight, hoodRight, turretRight);
    shooterTunerLeft = new ShooterTuner(flywheelLeft, hoodLeft, turretLeft);

    shooterHandlerRight =
        new ShooterHandler(
            turretRight,
            hoodRight,
            flywheelRight,
            indexerRight,
            robotContainer.drivetrain,
            RobotBase.isReal() ? ShooterConfig.actualConfig() : ShooterConfig.testConfig(),
            Setpoint.Side.RIGHT);

    shooterHandlerLeft =
        new ShooterHandler(
            turretLeft,
            hoodLeft,
            flywheelLeft,
            indexerLeft,
            robotContainer.drivetrain,
            RobotBase.isReal() ? ShooterConfig.actualConfig() : ShooterConfig.testConfig(),
            Setpoint.Side.LEFT);

    setGoal(SetpointGoal.NEUTRAL);
    this.goal = Goal.TUNER;
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {
      setGoal(SetpointGoal.NEUTRAL);

      if (Controllers.INTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE);
      }

      if (Controllers.XBOX.button(8).getAsBoolean()) {
        goal = Goal.MANUAL;
      }
      if (Controllers.XBOX.x().getAsBoolean()) {
        goal = Goal.SHOOT;
      }
      if (Controllers.XBOX.y().getAsBoolean()) {
        goal = Goal.TUNER;
      }
    }

    switch (goal) {
      case MANUAL -> {
        shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);

        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);

        setGoal(SetpointGoal.RESET);
      }
      case SHOOT -> {
        shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.SHOOT);

        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);
      }
      case TUNER -> {
        shooterTunerRight.setGoal(ShooterTuner.Goal.ACTIVE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);

        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);

        setGoal(SetpointGoal.INDEX);
        setGoal(SetpointGoal.INTAKE);
      }
    }

    if (goal == Goal.SHOOT
        && (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
            || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING)) {
      kicker.setVoltage(SetpointGoal.SHOOT.getSetpoint().getKicker().get());
    } else {
      kicker.setVoltage(SetpointGoal.NEUTRAL.getSetpoint().getKicker().get());
    }

    shooterHandlerLeft.periodic();
    shooterHandlerRight.periodic();

    // subsystems
    flywheelRight.periodic();
    flywheelLeft.periodic();
    hoodRight.periodic();
    hoodLeft.periodic();
    indexerLeft.periodic();
    indexerRight.periodic();
    turretRight.periodic();
    turretLeft.periodic();
    groundPivot.periodic();
    kicker.periodic();
    groundRollers.periodic();
  }

  public void setGoal(Setpoint setpoint) {}

  public void setGoal(SetpointGoal setpoint) {
    setGoal(setpoint.getSetpoint());
  }

  public void resetPositions() {
    hoodRight.resetPosition(SetpointGoal.RESET.getSetpoint().getSide(Side.RIGHT).getHood().get());
    turretRight.resetPosition(
        SetpointGoal.RESET.getSetpoint().getSide(Side.RIGHT).getTurret().get());
    hoodLeft.resetPosition(SetpointGoal.RESET.getSetpoint().getSide(Side.LEFT).getHood().get());
    turretLeft.resetPosition(SetpointGoal.RESET.getSetpoint().getSide(Side.LEFT).getTurret().get());
    groundPivot.resetPosition(SetpointGoal.RESET.getSetpoint().getGroundPivot().get());
  }

  public Command neutral() {
    return Commands.run(() -> setGoal(SetpointGoal.NEUTRAL));
  }

  public boolean freezeDriving() {
    return goal == Goal.TUNER
        && (shooterTunerRight.freezeDriving() || shooterTunerLeft.freezeDriving());
  }
}
