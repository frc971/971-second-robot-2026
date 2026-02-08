package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.ShooterConfigs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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

  public final CommandSwerveDrivetrain drivetrain;

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
  public final Climber climber;

  public final Visualization visualization;
  @AutoLogOutput @Getter private ShooterGoal shooterGoal = ShooterGoal.NONE;

  // If the turret is not within the interval (-SWAP_BUFFER, SWAP_BUFFER) one of the two turrets
  // will take a different arc to prevent collisions in the air
  private static final Angle SWAP_BUFFER = Degrees.of(30);

  private enum ShooterGoal {
    NONE,
    MANUAL_BOTH,
    MANUAL_LEFT,
    MANUAL_RIGHT,
    SHOOT_BOTH,
    SHOOT_LEFT,
    SHOOT_RIGHT,
    TUNER_LEFT,
    TUNER_RIGHT
  }

  private boolean prevIntakePivotBtn = false;
  private boolean intakeLowered = false;

  public Superstructure(RobotContainer robotContainer) {
    drivetrain = robotContainer.drivetrain;
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
    climber = new Climber();

    shooterTunerRight = new ShooterTuner(flywheelRight, hoodRight, turretRight);
    shooterTunerLeft = new ShooterTuner(flywheelLeft, hoodLeft, turretLeft);

    shooterHandlerRight =
        new ShooterHandler(
            turretRight,
            hoodRight,
            flywheelRight,
            robotContainer.drivetrain,
            ShooterConfigs.RIGHT_LOW);

    shooterHandlerLeft =
        new ShooterHandler(
            turretLeft, hoodLeft, flywheelLeft, robotContainer.drivetrain, ShooterConfigs.LEFT_LOW);

    visualization = new Visualization(turretLeft, turretRight, hoodLeft, hoodRight);

    setGoal(SetpointGoal.NEUTRAL);
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {
      setGoal(SetpointGoal.NEUTRAL);

      // Intake roller logic
      if (Controllers.OUTTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.OUTTAKE);
      } else if (Controllers.INTAKE_ROLLERS.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE_ROLLERS);
      }

      // Intake pivot logic
      boolean intakePivotBtn = Controllers.INTAKE_PIVOT.getAsBoolean();

      if (!prevIntakePivotBtn && intakePivotBtn) {
        intakeLowered = !intakeLowered;
      }

      if (intakeLowered) {
        setGoal(SetpointGoal.INTAKE_PIVOT);
      }

      prevIntakePivotBtn = intakePivotBtn;

      // Climber logic
      if (Controllers.XBOX.button(6).getAsBoolean()) {
        setGoal(SetpointGoal.RETRACT);
      }

      if (Controllers.XBOX.button(7).getAsBoolean()) {
        setGoal(SetpointGoal.EXTEND);
      }

      // Shooter logic
      if (Controllers.TUNE_LEFT.getAsBoolean()) {
        shooterGoal = ShooterGoal.TUNER_LEFT;
      }

      if (Controllers.TUNE_RIGHT.getAsBoolean()) {
        shooterGoal = ShooterGoal.TUNER_RIGHT;
      }

      if (Controllers.SHOOT_BOTH.getAsBoolean()) {
        shooterGoal = ShooterGoal.SHOOT_BOTH;
      }

      if (Controllers.SHOOT_LEFT.getAsBoolean()) {
        shooterGoal = ShooterGoal.SHOOT_LEFT;
      }

      if (Controllers.SHOOT_RIGHT.getAsBoolean()) {
        shooterGoal = ShooterGoal.SHOOT_RIGHT;
      }

      if (Controllers.MANUAL_BOTH.getAsBoolean()) {
        shooterGoal = ShooterGoal.MANUAL_BOTH;
      }

      if (Controllers.MANUAL_LEFT.getAsBoolean()) {
        shooterGoal = ShooterGoal.MANUAL_LEFT;
      }

      if (Controllers.MANUAL_RIGHT.getAsBoolean()) {
        shooterGoal = ShooterGoal.MANUAL_RIGHT;
      }
    }

    switch (shooterGoal) {
      case NONE, MANUAL_LEFT, MANUAL_RIGHT, MANUAL_BOTH -> {
        shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);

        switch (shooterGoal) {
          case MANUAL_LEFT -> setGoal(SetpointGoal.MANUAL_LEFT);
          case MANUAL_RIGHT -> setGoal(SetpointGoal.MANUAL_RIGHT);
          case MANUAL_BOTH -> setGoal(SetpointGoal.MANUAL_RIGHT);
          default -> {
            break;
          }
        }
      }
      case SHOOT_BOTH, SHOOT_LEFT, SHOOT_RIGHT -> {
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);
        shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);

        shooterHandlerLeft.setPhysics(ShooterConfigs.LEFT_HIGH.PHYSICS());
        shooterHandlerRight.setPhysics(ShooterConfigs.RIGHT_HIGH.PHYSICS());

        switch (shooterGoal) {
          case SHOOT_LEFT -> {
            shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.SHOOT);
            shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
            if (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING) {
              setGoal(SetpointGoal.LEFT_ONLY);
            }
          }
          case SHOOT_RIGHT -> {
            shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
            shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.SHOOT);
            if (shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING) {
              setGoal(SetpointGoal.RIGHT_ONLY);
            }
          }
          case SHOOT_BOTH -> {
            if (shooterHandlerRight.getRelativeTurretAngle().gt(SWAP_BUFFER)) {
              shooterHandlerRight.setPhysics(ShooterConfigs.RIGHT_HIGH.PHYSICS());
              shooterHandlerLeft.setPhysics(ShooterConfigs.LEFT_LOW.PHYSICS());
            } else if (shooterHandlerRight.getRelativeTurretAngle().lt(SWAP_BUFFER.unaryMinus())) {
              shooterHandlerRight.setPhysics(ShooterConfigs.RIGHT_LOW.PHYSICS());
              shooterHandlerLeft.setPhysics(ShooterConfigs.LEFT_HIGH.PHYSICS());
            }

            shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.SHOOT);
            shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.SHOOT);
            if (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
                && shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING) {
              setGoal(SetpointGoal.BOTH_SHOOT);
            } else if (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING) {
              setGoal(SetpointGoal.LEFT_ONLY);
            } else if (shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING) {
              setGoal(SetpointGoal.RIGHT_ONLY);
            }
          }
          default -> {
            break;
          }
        }
      }
      case TUNER_LEFT, TUNER_RIGHT -> {
        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);

        shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);
        shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);
        switch (shooterGoal) {
          case TUNER_LEFT -> {
            shooterTunerRight.setGoal(ShooterTuner.Goal.NONE);
            shooterTunerLeft.setGoal(ShooterTuner.Goal.ACTIVE);
            setGoal(SetpointGoal.LEFT_ONLY);
          }
          case TUNER_RIGHT -> {
            shooterTunerRight.setGoal(ShooterTuner.Goal.ACTIVE);
            shooterTunerLeft.setGoal(ShooterTuner.Goal.NONE);
            setGoal(SetpointGoal.RIGHT_ONLY);
          }
          default -> {
            break;
          }
        }

        if (Controllers.OUTTAKE.getAsBoolean()) {
          setGoal(SetpointGoal.OUTTAKE);
        } else {
          setGoal(SetpointGoal.INDEX);
          setGoal(SetpointGoal.INTAKE_ROLLERS);
        }
      }
    }

    if (Controllers.OUTTAKE.getAsBoolean()) {
      kicker.setVoltage(SetpointGoal.OUTTAKE.getSetpoint().getKicker().get());
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
    kicker.periodic();
    climber.periodic();

    visualization.periodic();
  }

  public void setGoal(Setpoint setpoint) {
    if (setpoint.getLeftIndexer().isPresent()) {
      indexerLeft.setVoltage(setpoint.getLeftIndexer().get());
    }
    if (setpoint.getRightIndexer().isPresent()) {
      indexerRight.setVoltage(setpoint.getRightIndexer().get());
    }

    if (setpoint.getGroundPivot().isPresent()) {
      groundPivot.setPosition(setpoint.getGroundPivot().get());
    }
    if (setpoint.getGroundRollers().isPresent()) {
      groundRollers.setVoltage(setpoint.getGroundRollers().get());
    }
    if (setpoint.getKicker().isPresent()) {
      kicker.setVoltage(setpoint.getKicker().get());
    }
    // left
    if (setpoint.getLeftFlywheel().isPresent()) {
      flywheelLeft.setVelocity(setpoint.getLeftFlywheel().get());
    }
    if (setpoint.getLeftHood().isPresent()) {
      hoodLeft.setPosition(setpoint.getLeftHood().get());
    }
    if (setpoint.getLeftTurret().isPresent()) {
      turretLeft.setPosition(setpoint.getLeftTurret().get());
    }
    if (setpoint.getLeftIndexer().isPresent()) {
      indexerLeft.setVoltage(setpoint.getLeftIndexer().get());
    }
    // right
    if (setpoint.getRightFlywheel().isPresent()) {
      flywheelRight.setVelocity(setpoint.getRightFlywheel().get());
    }
    if (setpoint.getRightHood().isPresent()) {
      hoodRight.setPosition(setpoint.getRightHood().get());
    }
    if (setpoint.getRightTurret().isPresent()) {
      turretRight.setPosition(setpoint.getRightTurret().get());
    }
    if (setpoint.getRightIndexer().isPresent()) {
      indexerRight.setVoltage(setpoint.getRightIndexer().get());
    }

    if (setpoint.getClimber().isPresent()) {
      climber.setPosition(setpoint.getClimber().get());
    }
  }

  public void setGoal(SetpointGoal setpoint) {
    setGoal(setpoint.getSetpoint());
  }

  public void resetPositions() {
    hoodRight.resetPosition(SetpointGoal.RESET.getSetpoint().getRightHood().get());
    turretRight.resetPosition(SetpointGoal.RESET.getSetpoint().getRightTurret().get());
    hoodLeft.resetPosition(SetpointGoal.RESET.getSetpoint().getLeftHood().get());
    turretLeft.resetPosition(SetpointGoal.RESET.getSetpoint().getLeftTurret().get());
    groundPivot.resetPosition(SetpointGoal.RESET.getSetpoint().getGroundPivot().get());
    climber.resetPosition(SetpointGoal.RESET.getSetpoint().getClimber().get());
  }

  public Command neutral() {
    return Commands.run(() -> setGoal(SetpointGoal.NEUTRAL));
  }

  public boolean freezeDriving() {
    return (shooterTunerRight.freezeDriving() || shooterTunerLeft.freezeDriving());
  }
}
