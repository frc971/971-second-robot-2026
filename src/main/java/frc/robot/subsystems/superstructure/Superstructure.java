package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

  public final Indexer indexer;
  public final GroundRollers groundRollers;
  public final GroundPivot groundPivot;

  public final ShooterHandler shooterHandlerRight;
  public final ShooterHandler shooterHandlerLeft;

  public final TurretRight turretRight;
  public final TurretLeft turretLeft;

  public final Climber climber;

  public final Visualization visualization;
  @AutoLogOutput private ShooterGoal shooterGoal = ShooterGoal.NONE;

  private boolean pivotAtPosition = false;
  private final Timer pivotTimer = new edu.wpi.first.wpilibj.Timer();
  private boolean pivotTimerStarted = false;

  private enum ShooterGoal {
    NONE,
    MANUAL,
    TARGETING,
    TUNE_LEFT_SHOOTER,
    TUNE_RIGHT_SHOOTER,
  }

  public Superstructure(RobotContainer robotContainer) {
    drivetrain = robotContainer.drivetrain;
    flywheelRight = new FlywheelRight();
    flywheelLeft = new FlywheelLeft();
    hoodRight = new HoodRight();
    hoodLeft = new HoodLeft();
    indexer = new Indexer();
    turretRight = new TurretRight();
    turretLeft = new TurretLeft();
    groundPivot = new GroundPivot();
    groundRollers = new GroundRollers();
    climber = new Climber();

    shooterHandlerRight =
        new ShooterHandler(
            turretRight,
            hoodRight,
            flywheelRight,
            robotContainer.drivetrain,
            ShooterConfigs.RIGHT,
            ShooterHandler.Side.RIGHT);

    shooterHandlerLeft =
        new ShooterHandler(
            turretLeft,
            hoodLeft,
            flywheelLeft,
            robotContainer.drivetrain,
            ShooterConfigs.LEFT,
            ShooterHandler.Side.LEFT);

    visualization =
        new Visualization(turretLeft, turretRight, hoodLeft, hoodRight, climber, groundPivot);

    setGoal(SetpointGoal.NEUTRAL);
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {
      holdTeleopSafeState();

      // Climber logic
      if (Controllers.CLIMB_RETRACT.getAsBoolean()) {
        setGoal(SetpointGoal.RETRACT);
      } else if (Controllers.CLIMB_EXTEND.getAsBoolean()) {
        setGoal(SetpointGoal.EXTEND);
      }

      boolean wantsShot =
          Controllers.LEFT_SHUTTLE.getAsBoolean()
              || Controllers.RIGHT_SHUTTLE.getAsBoolean()
              || Controllers.SHOOT.getAsBoolean()
              || Controllers.SHOOT_REDUNDANCY.getAsBoolean();

      // switch MANUAL, TUNING, TARGETING (currently don't deal with NONE)
      if (Controllers.MANUAL.toggled()) {
        shooterGoal = ShooterGoal.MANUAL;
      } else if (wantsShot) {
        shooterGoal = ShooterGoal.TARGETING;
      } else {
        shooterGoal = ShooterGoal.NONE;
      }

      shooterHandlerLeft.setUseTOF(!Controllers.DISABLE_OTF.getAsBoolean());
      shooterHandlerRight.setUseTOF(!Controllers.DISABLE_OTF.getAsBoolean());

      shooterHandlerLeft.setTuningEnabled(Controllers.TUNE_LEFT.getAsBoolean());
      shooterHandlerRight.setTuningEnabled(Controllers.TUNE_RIGHT.getAsBoolean());

      shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
      shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);

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

          if (Controllers.MANUAL_RESET.toggled()) {
            setpoint = SetpointGoal.MANUAL_RESET;
          }

          setGoal(setpoint);
        }
        case TUNE_LEFT_SHOOTER -> {}
        case TUNE_RIGHT_SHOOTER -> {}
      }

      if (Controllers.INTAKE_PIVOT.toggled()) {
        setGoal(SetpointGoal.INTAKE_PIVOT);
      } else {
        groundPivot.setPosition(SetpointGoal.NEUTRAL.getSetpoint().getGroundPivot().get());
      }

      if (Controllers.INTAKE_ROLLERS.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE_ROLLERS);
      }

      if (wantsShot && DriverStation.isEnabled()) {
        if (!Controllers.KILL_LEFT.toggled()
            && shooterHandlerLeft.getDesiredHoodAngle().isPresent()) {
          hoodLeft.setPosition(shooterHandlerLeft.getDesiredHoodAngle().get());
        }

        if (!Controllers.KILL_RIGHT.toggled()
            && shooterHandlerRight.getDesiredHoodAngle().isPresent()) {
          hoodRight.setPosition(shooterHandlerRight.getDesiredHoodAngle().get());
        }
      }

      // Indexer Logic
      // Driver has to say we can shoot AND we need to be ready to shoot
      boolean indexing =
          wantsShot
              && ((shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
                      || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING)
                  || shooterGoal == ShooterGoal.MANUAL);

      if (indexing) {
        setGoal(SetpointGoal.INDEX);
      } else if (Controllers.OUTTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.OUTTAKE);
      }

      // Killing turret logicb 
      if (Controllers.KILL_LEFT.toggled()) {
        setGoal(SetpointGoal.KILL_LEFT.getSetpoint());
      }
      if (Controllers.KILL_RIGHT.toggled()) {
        setGoal(SetpointGoal.KILL_RIGHT.getSetpoint());
      }

      if (Controllers.SUPERCHARGED.getAsBoolean()) {
        setGoal(SetpointGoal.SUPERCHARGED);
        shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
        shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
      }
    } else if (DriverStation.isAutonomous()) {
      if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.Goal.ACTIVE
          && shooterHandlerLeft.getDesiredHoodAngle().isPresent()) {
        hoodLeft.setPosition(shooterHandlerLeft.getDesiredHoodAngle().get());
      }

      if (shooterHandlerRight.getShooterGoal() == ShooterHandler.Goal.ACTIVE
          && shooterHandlerRight.getDesiredHoodAngle().isPresent()) {
        hoodRight.setPosition(shooterHandlerRight.getDesiredHoodAngle().get());
      }

      if (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
          || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING) {
        setGoal(SetpointGoal.INDEX);
      }
    }

    shooterHandlerLeft.periodic();
    shooterHandlerRight.periodic();

    // subsystems
    flywheelRight.periodic();
    flywheelLeft.periodic();
    hoodRight.periodic();
    hoodLeft.periodic();
    indexer.periodic();
    turretRight.periodic();
    turretLeft.periodic();
    groundPivot.periodic();
    groundRollers.periodic();
    climber.periodic();

    visualization.periodic();
  }

  public void setGoal(Setpoint setpoint) {
    if (setpoint.getIndexer().isPresent()) {
      indexer.setVoltage(setpoint.getIndexer().get());
    }

    if (setpoint.getGroundPivot().isPresent()) {
      groundPivot.setPosition(setpoint.getGroundPivot().get());
    }
    if (setpoint.getGroundRollers().isPresent()) {
      groundRollers.setVoltage(setpoint.getGroundRollers().get());
    }
    // left
    if (setpoint.getLeftFlywheel().isPresent()) {
      flywheelLeft.setVelocity(
          setpoint.getLeftFlywheel().get().plus(shooterHandlerLeft.getFlywheelOffset()));
    }
    if (setpoint.getLeftHood().isPresent()) {
      hoodLeft.setPosition(setpoint.getLeftHood().get());
    }
    if (setpoint.getLeftTurret().isPresent()) {
      turretLeft.setPosition(
          setpoint.getLeftTurret().get().plus(shooterHandlerLeft.getTurretOffset()));
    }
    // right
    if (setpoint.getRightFlywheel().isPresent()) {
      flywheelRight.setVelocity(
          setpoint.getRightFlywheel().get().plus(shooterHandlerRight.getFlywheelOffset()));
    }
    if (setpoint.getRightHood().isPresent()) {
      hoodRight.setPosition(setpoint.getRightHood().get());
    }
    if (setpoint.getRightTurret().isPresent()) {
      turretRight.setPosition(
          setpoint.getRightTurret().get().plus(shooterHandlerRight.getTurretOffset()));
    }
    if (setpoint.getClimber().isPresent()) {
      climber.setPosition(setpoint.getClimber().get());
    }
  }

  public void setGoal(SetpointGoal setpoint) {
    setGoal(setpoint.getSetpoint());
  }

  private void holdTeleopSafeState() {
    flywheelLeft.setVelocity(RotationsPerSecond.zero());
    flywheelRight.setVelocity(RotationsPerSecond.zero());

    hoodLeft.setPosition(hoodLeft.getPosition());
    hoodRight.setPosition(hoodRight.getPosition());
    turretLeft.setPosition(turretLeft.getPosition());
    turretRight.setPosition(turretRight.getPosition());

    indexer.setVoltage(Volts.zero());
    groundRollers.setVoltage(Volts.zero());
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
    return Commands.runOnce(
        () -> {
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
          setGoal(SetpointGoal.AUTO_NEUTRAL);
        });
  }

  public Command intakeAuto() {
    return Commands.runOnce(
        () -> {
          setGoal(SetpointGoal.AUTO_INTAKE_ROLLERS);
        });
  }

  public Command intakePivotDownAuto() {
    return Commands.race(
        Commands.run(
            () -> {
              groundPivot.setVoltage(Volts.of(-4.0));
            }),
        Commands.waitSeconds(2));
  }

  public Command deployedAuto() {
    return Commands.runOnce(
        () -> {
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
          setGoal(SetpointGoal.AUTO_NEUTRAL);
        });
  }

  public Command shootAuto() {
    return Commands.runOnce(
        () -> {
          ObjectState curTarget =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? ShooterHandler.Targets.BLUE
                  : ShooterHandler.Targets.RED;
          shooterHandlerLeft.setTargetState(curTarget);
          shooterHandlerRight.setTargetState(curTarget);

          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);
        });
  }

  public Command shootSequenceAuto() {
    return Commands.run(
        () -> {
          ObjectState curTarget =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? ShooterHandler.Targets.BLUE
                  : ShooterHandler.Targets.RED;
          shooterHandlerLeft.setTargetState(curTarget);
          shooterHandlerRight.setTargetState(curTarget);
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);
        });
  }
}
