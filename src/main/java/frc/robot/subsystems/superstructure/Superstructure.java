package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfigs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Central place to instantiate and hold references to robot mechanism subsystems. This prevents
 * {@code RobotContainer} from being cluttered with individual mechanism construction logic and
 * makes it easier to expand later.
 */
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

  public final ShooterTuner shooterTuner;

  public final ShooterHandler shooterHandlerRight;
  public final ShooterHandler shooterHandlerLeft;

  public final TurretRight turretRight;
  public final TurretLeft turretLeft;

  public final Kicker kicker;
  public final Climber climber;

  public final Visualization visualization;
  @AutoLogOutput private ShooterGoal shooterGoal = ShooterGoal.NONE;

  // If the turret is not within the interval (-SWAP_BUFFER, SWAP_BUFFER) one of the two turrets
  // will take a different arc to prevent collisions in the air
  private static final Angle SWAP_BUFFER = Degrees.of(30);

  private enum ShooterGoal {
    NONE,
    MANUAL,
    TARGETING,
    TUNE_LEFT_SHOOTER,
    TUNE_RIGHT_SHOOTER,
  }

  private boolean freezeIntake = false;
  private boolean prevIntakePressed = false;

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

    shooterHandlerRight =
        new ShooterHandler(
            turretRight, hoodRight, flywheelRight, robotContainer.drivetrain, ShooterConfigs.RIGHT);

    shooterHandlerLeft =
        new ShooterHandler(
            turretLeft, hoodLeft, flywheelLeft, robotContainer.drivetrain, ShooterConfigs.LEFT);

    // default left turret
    shooterTuner = new ShooterTuner(flywheelLeft, hoodLeft, turretLeft, shooterHandlerLeft);

    visualization =
        new Visualization(turretLeft, turretRight, hoodLeft, hoodRight, climber, groundPivot);

    setGoal(SetpointGoal.NEUTRAL);
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {
      setGoal(SetpointGoal.NEUTRAL);

      // Climber logic
      if (Controllers.CLIMB_RETRACT.getAsBoolean()) {
        setGoal(SetpointGoal.RETRACT);
      } else if (Controllers.CLIMB_EXTEND.getAsBoolean()) {
        setGoal(SetpointGoal.EXTEND);
      }

      // switch MANUAL, TUNING, TARGETING (currently don't deal with NONE)
      if (Controllers.MANUAL.toggled()) {
        shooterGoal = ShooterGoal.MANUAL;
      } else if (Controllers.TUNING.toggled()) {
        shooterGoal = ShooterGoal.TUNE_LEFT_SHOOTER;
        // TODO: make something for right tuner?
      } else {
        shooterGoal = ShooterGoal.TARGETING;
      }

      // TODO: is this covered by the neutral setpoint or no?
      shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
      shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
      shooterTuner.setGoal(ShooterTuner.Goal.NONE);

      switch (shooterGoal) {
        case NONE -> {}
        case TARGETING -> {
          if (!Controllers.KILL_LEFT.toggled()) {
            shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);
          }
          if (!Controllers.KILL_RIGHT.toggled()) {
            shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
          }

          ObjectState curTarget =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? ShooterHandler.Targets.BLUE
                  : ShooterHandler.Targets.RED;
          if (Controllers.SHUTTLE_LEFT.getAsBoolean()) {
            curTarget =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterHandler.Targets.LEFT_BLUE_SHUTTLE
                    : ShooterHandler.Targets.LEFT_RED_SHUTTLE;
          }
          if (Controllers.SHUTTLE_RIGHT.getAsBoolean()) {
            curTarget =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterHandler.Targets.RIGHT_BLUE_SHUTTLE
                    : ShooterHandler.Targets.RIGHT_RED_SHUTTLE;
          }
          shooterHandlerLeft.setTargetState(curTarget);
          shooterHandlerRight.setTargetState(curTarget);
        }
        case MANUAL -> {
          SetpointGoal setpoint = SetpointGoal.NEUTRAL;

          if (Controllers.MANUAL_SHOOT_UP.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_UP;
          } else if (Controllers.MANUAL_SHOOT_DOWN.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_DOWN;
          } else if (Controllers.MANUAL_SHOOT_LEFT.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_LEFT;
          } else if (Controllers.MANUAL_SHOOT_RIGHT.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_RIGHT;
          } else if (Controllers.MANUAL_SHUTTLE_UP.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_SHUTTLE_UP;
          } else if (Controllers.MANUAL_SHUTTLE_DOWN.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_SHUTTLE_DOWN;
          } else if (Controllers.MANUAL_SHUTTLE_LEFT.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_SHUTTLE_LEFT;
          } else if (Controllers.MANUAL_SHUTTLE_RIGHT.getAsBoolean()) {
            setpoint = SetpointGoal.MANUAL_SHUTTLE_RIGHT;
          }

          setGoal(setpoint);
        }
        case TUNE_LEFT_SHOOTER -> {
          shooterTuner.setGoal(ShooterTuner.Goal.ACTIVE);

          if (shooterTuner.isIndexing()) {
            setGoal(SetpointGoal.INDEX_LEFT);
          }
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
        }
        case TUNE_RIGHT_SHOOTER -> {}
      }

      // intake (will override outtake)
      if (freezeIntake) {
        setGoal(SetpointGoal.INTAKE_PIVOT);
      }

      if (Controllers.INTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE_ROLLERS);
        setGoal(SetpointGoal.INTAKE_PIVOT);

        if (!prevIntakePressed) {
          freezeIntake = false;
        }

        if (Controllers.INTAKE_FREEZE.getAsBoolean()) {
          freezeIntake = true;
        }
      }

      prevIntakePressed = Controllers.INTAKE.getAsBoolean();

      // Indexer Logic
      // Driver has to say we can shoot AND we need to be ready to shoot
      boolean indexing =
          (Controllers.LEFT_SHUTTLE.getAsBoolean()
                  || Controllers.RIGHT_SHUTTLE.getAsBoolean()
                  || Controllers.SHOOT.getAsBoolean()
                  || Controllers.SHOOT_REDUNDANCY.getAsBoolean())
              && (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
                  || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING);

      if (indexing) {
        if (!Controllers.KILL_LEFT.toggled() && !Controllers.KILL_RIGHT.toggled()) {
          setGoal(SetpointGoal.INDEX_BOTH);
        } else if (Controllers.KILL_LEFT.toggled()) {
          setGoal(SetpointGoal.INDEX_RIGHT);
        } else if (Controllers.KILL_RIGHT.toggled()) {
          setGoal(SetpointGoal.INDEX_LEFT);
        }
      } else if (Controllers.OUTTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.OUTTAKE);
      }

      // Killing turret logic
      if (Controllers.KILL_LEFT.toggled()) {
        setGoal(SetpointGoal.KILL_LEFT.getSetpoint());
      }
      if (Controllers.KILL_RIGHT.toggled()) {
        setGoal(SetpointGoal.KILL_RIGHT.getSetpoint());
      }
    }

    shooterHandlerLeft.setTuningEnabled(Controllers.TUNE_LEFT.toggled());
    shooterHandlerRight.setTuningEnabled(Controllers.TUNE_RIGHT.toggled());

    shooterTuner.periodic();

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
    return (shooterTuner.freezeDriving())
        && (shooterGoal == ShooterGoal.TUNE_LEFT_SHOOTER
            || shooterGoal == ShooterGoal.TUNE_RIGHT_SHOOTER);
  }
}
