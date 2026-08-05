package frc.robot.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.ShooterHandler;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FuelSimHelper {

  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;
  private final ShooterHandler[] shooterHandlers;
  private final Voltage autoShootingGoal = Volts.of(11.0);
  private final double wheelDiameterM = Inches.of(4.0).in(Meters); // from shooter CAD

  // how much surface speed transfers to the ball, 0.45 prevents overshooting
  private final double slipFactor = 0.45;

  private double simFuelExitVelocity;

  // Pass in robotContainer in order to access superstructure and drivetrain
  public FuelSimHelper(RobotContainer robotContainer) {
    this.drivetrain = robotContainer.drivetrain;
    this.superstructure = robotContainer.superstructure;
    shooterHandlers =
        new ShooterHandler[] {
          superstructure.shooterHandlerLeft, superstructure.shooterHandlerRight
        };
  }

  public void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.clearFuel();
    instance.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        () -> drivetrain.getState().Pose,
        this::getFieldRelativeChassisSpeedsForSim);

    instance.registerIntake(
        -Dimensions.FULL_LENGTH,
        Dimensions.FULL_LENGTH / 2.0,
        -Dimensions.FULL_WIDTH / 6.0,
        Dimensions.FULL_WIDTH / 6.0,
        () -> (true),
        () -> Logger.recordOutput("Fuel Simulation/LastEvent", "Intake"));

    instance.spawnStartingFuel();
    instance.start();
  }

  public void resetFuelSim() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    FuelSim instance = FuelSim.getInstance();
    instance.clearFuel();
    instance.spawnStartingFuel();
    Logger.recordOutput("Fuel Simulation/LastEvent", "Auto Reset");
  }

  private void launchFuelInSim(
      LinearVelocity velocity, Angle elevation, ShooterHandler shooterHandler) {
    Translation3d muzzlePose = shooterHandler.getProjectileState().position();
    Rotation2d launchYaw =
        drivetrain
            .getState()
            .Pose
            .getRotation()
            .plus(new Rotation2d(shooterHandler.getTurret().getPosition()));

    Translation3d launchVelocity = createLaunchVelocity(velocity, elevation, launchYaw);
    launchVelocity = launchVelocity.plus(shooterHandler.getProjectileState().velocity());

    FuelSim.getInstance().spawnFuel(muzzlePose, launchVelocity);

    Logger.recordOutput("Fuel Simulation/LastEvent", "Shoot");
  }

  @AutoLogOutput(key = "Fuel Simulation/Shooting/isShooting")
  private boolean isShooting() {
    boolean shooting = false;

    Voltage rollerFloorVolts = superstructure.rollerFloor.getAppliedVoltage();
    Voltage b2Volts = superstructure.b2.getAppliedVoltage();
    Voltage kickerVolts = superstructure.kicker.getAppliedVoltage();

    if (rollerFloorVolts.magnitude() > 0 && b2Volts.magnitude() > 0 && kickerVolts.magnitude() > 0) {
      shooting = true;
    }

    return shooting;
  }

  // credit to team 9562 for exitVelocity function
  private double exitVelocity(double rpm) {
    return slipFactor * rpm * Math.PI * wheelDiameterM / 60.0;
  }

  private void handleSimShooting() {
    if (isShooting()) {
      for (ShooterHandler shooterHandler : shooterHandlers) {
        Optional<AngularVelocity> flywheelSpeed = shooterHandler.getFlywheelSpeed();
        Optional<Angle> hoodAngle = shooterHandler.getHoodAngle();

        if (flywheelSpeed.isPresent() && hoodAngle.isPresent()) {
          simFuelExitVelocity = exitVelocity(flywheelSpeed.get().in(RPM));
          launchFuelInSim(MetersPerSecond.of(simFuelExitVelocity), hoodAngle.get(), shooterHandler);
        }
      }
    }
  }

  private Translation3d createLaunchVelocity(
      LinearVelocity velocity, Angle elevation, Rotation2d heading) {
    double speed = velocity.in(MetersPerSecond);
    double elevationRadians = elevation.in(Radians);
    double planarSpeed = speed * Math.cos(elevationRadians);
    double verticalSpeed = speed * Math.sin(elevationRadians);
    Translation2d planar = new Translation2d(planarSpeed, 0).rotateBy(heading);
    return new Translation3d(planar.getX(), planar.getY(), verticalSpeed);
  }

  private ChassisSpeeds getFieldRelativeChassisSpeedsForSim() {
    ChassisSpeeds speeds = drivetrain.getState().Speeds;
    return (speeds == null) ? new ChassisSpeeds() : speeds;
  }

  public void periodic() {
    Logger.recordOutput("Fuel Simulation/Shooting/ExitVelocity", simFuelExitVelocity);
    handleSimShooting();
  }
}
