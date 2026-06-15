package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.subsystems.superstructure.ShooterHandler.State;
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

  public final RollerFloor rollerFloor;
  public final B2 b2;
  public final Kicker kicker;

  public final GroundRollers groundRollers;
  public final GroundPivot groundPivot;

  public final ShooterHandler shooterHandlerRight;
  public final ShooterHandler shooterHandlerLeft;

  public final TurretRight turretRight;
  public final TurretLeft turretLeft;

  public final Visualization visualization;
  @AutoLogOutput private ShooterGoal shooterGoal = ShooterGoal.NONE;

  private final Timer juiceTimer = new Timer();
  private boolean juiceAuto = false;

  private enum ShooterGoal {
    NONE,
    MANUAL,
    TARGETING
  }

  public Superstructure(RobotContainer robotContainer) {
    drivetrain = robotContainer.drivetrain;
    flywheelRight = new FlywheelRight();
    flywheelLeft = new FlywheelLeft();
    hoodRight = new HoodRight();
    hoodLeft = new HoodLeft();
    turretRight = new TurretRight();
    turretLeft = new TurretLeft();
    groundPivot = new GroundPivot();
    groundRollers = new GroundRollers();
    rollerFloor = new RollerFloor();
    b2 = new B2();
    kicker = new Kicker();

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

    visualization = new Visualization(turretLeft, turretRight, hoodLeft, hoodRight, groundPivot);

    setGoal(SetpointGoal.NEUTRAL);
  }

  public void periodic() {
    // MARK: Teleop Logic
    if (DriverStation.isTeleop()) {
      if (!juiceTimer.isRunning()) {
        juiceTimer.restart();
      }
      setGoal(SetpointGoal.NEUTRAL);

      boolean wantsShot =
          Controllers.LEFT_SHUTTLE.getAsBoolean()
              || Controllers.RIGHT_SHUTTLE.getAsBoolean()
              || Controllers.SHOOT.getAsBoolean()
              || Controllers.SHOOT_REDUNDANCY.getAsBoolean();

      // switch MANUAL, TUNING, TARGETING (currently don't deal with NONE)
      if (Controllers.MANUAL.toggled()) {
        shooterGoal = ShooterGoal.MANUAL;
      } else {
        shooterGoal = ShooterGoal.TARGETING;
      }

      shooterHandlerLeft.setUseOTF(!Controllers.DISABLE_OTF.getAsBoolean());
      shooterHandlerRight.setUseOTF(!Controllers.DISABLE_OTF.getAsBoolean());

      shooterHandlerLeft.setTuningEnabled(Controllers.TUNE_LEFT.getAsBoolean());
      shooterHandlerRight.setTuningEnabled(Controllers.TUNE_RIGHT.getAsBoolean());

      shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
      shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);

      switch (shooterGoal) {
        case NONE -> {}
        case TARGETING -> {
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);

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
      }

      updateIntakePivotGoal();

      if (Controllers.INTAKE_ROLLERS.getAsBoolean()) {
        setGoal(SetpointGoal.INTAKE_ROLLERS);
        groundPivot.setFeedforward(Volts.of(-0.4));
      } else {
        groundPivot.setFeedforward(Volts.of(0.0));
      }

      if (wantsShot && DriverStation.isEnabled()) {
        shooterHandlerLeft.getHoodAngle().ifPresent(hoodLeft::setPosition);
        shooterHandlerLeft
            .getFlywheelSpeed()
            .ifPresent(
                speed ->
                    flywheelLeft.setVelocity(speed.plus(shooterHandlerLeft.getFlywheelOffset())));

        shooterHandlerRight.getHoodAngle().ifPresent(hoodRight::setPosition);
        shooterHandlerRight
            .getFlywheelSpeed()
            .ifPresent(
                speed ->
                    flywheelRight.setVelocity(speed.plus(shooterHandlerRight.getFlywheelOffset())));
      }

      // Indexer Logic
      // Driver has to say we can shoot AND we need to be ready to shoot
      if (!wantsShot && shooterHandlerLeft.getShooterState() == State.FIRING) {
        shooterHandlerLeft.setStateAiming();
      }

      if (!wantsShot && shooterHandlerRight.getShooterState() == State.FIRING) {
        shooterHandlerRight.setStateAiming();
      }

      boolean indexing =
          wantsShot
              && ((shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
                      || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING)
                  || shooterGoal == ShooterGoal.MANUAL);

      if (Controllers.OUTTAKE.getAsBoolean()) {
        setGoal(SetpointGoal.OUTTAKE);
      } else if (Controllers.UNJAM.getAsBoolean()) {
        setGoal(SetpointGoal.UNJAM);
      } else if (indexing) {
        setGoal(SetpointGoal.INDEX);
      }

    } else if (DriverStation.isAutonomousEnabled()) { // MARK: Autonomous Logic
      if (!juiceTimer.isRunning()) {
        juiceTimer.restart();
      }

      if (juiceAuto) {
        int t = (int) (juiceTimer.get() * 100);

        if (t % 100 < 50) {
          setGoal(SetpointGoal.INTAKE_PIVOT);
        } else {
          setGoal(SetpointGoal.INTAKE_PIVOT_JUICE);
        }
      }

      if (shooterHandlerLeft.getShooterGoal() == ShooterHandler.Goal.ACTIVE) {
        shooterHandlerLeft.getHoodAngle().ifPresent(hoodLeft::setPosition);
        shooterHandlerLeft
            .getFlywheelSpeed()
            .ifPresent(
                speed ->
                    flywheelLeft.setVelocity(speed.plus(shooterHandlerLeft.getFlywheelOffset())));
      }

      if (shooterHandlerRight.getShooterGoal() == ShooterHandler.Goal.ACTIVE) {
        shooterHandlerRight.getHoodAngle().ifPresent(hoodRight::setPosition);
        shooterHandlerRight
            .getFlywheelSpeed()
            .ifPresent(
                speed ->
                    flywheelRight.setVelocity(speed.plus(shooterHandlerRight.getFlywheelOffset())));
      }

      if (shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING
          || shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING) {
        setGoal(SetpointGoal.AUTO_INDEX);
      } else {
        setGoal(SetpointGoal.AUTO_STOP_INDEXING);
      }

      setGoal(SetpointGoal.AUTO_INTAKE_ROLLERS);
    }

    shooterHandlerLeft.periodic();
    shooterHandlerRight.periodic();

    // subsystems
    flywheelRight.periodic();
    flywheelLeft.periodic();
    hoodRight.periodic();
    hoodLeft.periodic();
    rollerFloor.periodic();
    b2.periodic();
    kicker.periodic();
    turretRight.periodic();
    turretLeft.periodic();
    groundPivot.periodic();
    groundRollers.periodic();

    visualization.periodic();
  }

  // MARK: Helper functions

  private void updateIntakePivotGoal() {
    if (Controllers.INTAKE_PIVOT.toggled()) {
      SetpointGoal intakePivotGoal = SetpointGoal.INTAKE_PIVOT_JUICE;

      if (Controllers.JUICE.getAsBoolean()) {
        int t = (int) (juiceTimer.get() * 100);

        if (t % 100 >= 50) {
          intakePivotGoal = SetpointGoal.INTAKE_PIVOT;
        }
      }

      setGoal(intakePivotGoal);
    }
  }

  public void setGoal(Setpoint setpoint) {
    if (setpoint.getRollerFloor().isPresent()) {
      rollerFloor.setVoltage(setpoint.getRollerFloor().get());
    }
    if (setpoint.getB2().isPresent()) {
      b2.setVoltage(setpoint.getB2().get());
    }
    if (setpoint.getKicker().isPresent()) {
      kicker.setVoltage(setpoint.getKicker().get());
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
  }

  // MARK: AUTO Commands

  public Command neutral() {
    return Commands.runOnce(
        () -> {
          juiceAuto = false;
          shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.NONE);
          shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.NONE);
          setGoal(SetpointGoal.AUTO_NEUTRAL);
        });
  }

  public Command intakePivotDownAuto() {
    return Commands.runOnce(
        () -> {
          juiceAuto = false;
          setGoal(SetpointGoal.INTAKE_PIVOT);
        });
  }

  public Command reverseShooters() {
    return Commands.run(
            () -> {
              setGoal(SetpointGoal.REVERSE_SHOOTERS);
            })
        .withTimeout(1)
        .andThen(
            () -> {
              setGoal(SetpointGoal.AUTO_NEUTRAL);
            })
        .withTimeout(1);
  }

  public Command shootAuto() {
    return Commands.waitUntil(() -> !drivetrain.isRobotOnBump())
        .andThen(
            Commands.runOnce(
                () -> {
                  ObjectState curTarget =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? ShooterHandler.Targets.BLUE
                          : ShooterHandler.Targets.RED;
                  shooterHandlerLeft.setTargetState(curTarget);
                  shooterHandlerRight.setTargetState(curTarget);

                  shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
                  shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);

                  juiceAuto = true;
                }));
  }

  public Command shootAutoNoJuice() {
    return Commands.waitUntil(() -> !drivetrain.isRobotOnBump())
        .andThen(
            Commands.runOnce(
                () -> {
                  ObjectState curTarget =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? ShooterHandler.Targets.BLUE
                          : ShooterHandler.Targets.RED;
                  shooterHandlerLeft.setTargetState(curTarget);
                  shooterHandlerRight.setTargetState(curTarget);

                  shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
                  shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);

                  juiceAuto = false;
                }));
  }

  public Command shootSequenceAuto() {
    return Commands.run(
        () -> {
          groundPivot.setVoltage(Volts.of(2.0));
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

  public Command shootOnceAuto() {
    return Commands.waitUntil(() -> !drivetrain.isRobotOnBump())
        .andThen(
            Commands.runOnce(
                () -> {
                  ObjectState curTarget =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? ShooterHandler.Targets.BLUE
                          : ShooterHandler.Targets.RED;
                  shooterHandlerLeft.setTargetState(curTarget);
                  shooterHandlerRight.setTargetState(curTarget);

                  shooterHandlerRight.setShooterGoal(ShooterHandler.Goal.ACTIVE);
                  shooterHandlerLeft.setShooterGoal(ShooterHandler.Goal.ACTIVE);
                }));
  }
}
