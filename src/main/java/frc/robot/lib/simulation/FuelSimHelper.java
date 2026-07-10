package frc.robot.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotContainer;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.ShooterHandler;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class FuelSimHelper {

  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;
  private double simFuelExitVelocity;

  // Pass in robotContainer in order to access superstructure and drivetrain
  public FuelSimHelper(RobotContainer robotContainer) {
    this.drivetrain = robotContainer.drivetrain;
    this.superstructure = robotContainer.superstructure;
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
        () -> Logger.recordOutput("FuelSim/LastEvent", "Intake"));

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
    Logger.recordOutput("FuelSim/LastEvent", "Auto Reset");
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

    Logger.recordOutput("FuelSim/LastEvent", "Launch");
  }

  private void handleSimShooting() {
    if ((superstructure.shooterHandlerLeft.getShooterState() == ShooterHandler.State.FIRING)
        && (superstructure.shooterHandlerRight.getShooterState() == ShooterHandler.State.FIRING)) {

      ShooterHandler[] shooterHandlers =
          new ShooterHandler[] {
            superstructure.shooterHandlerLeft, superstructure.shooterHandlerRight
          };

      for (ShooterHandler shooterHandler : shooterHandlers) {
        LaunchSolution launchSolution = shooterHandler.getLaunchSolution();

        Angle hoodAngle = launchSolution.hoodAngle();

        simFuelExitVelocity = shooterHandler.getPhysics().getExitSpeed();

        launchFuelInSim(MetersPerSecond.of(simFuelExitVelocity), hoodAngle, shooterHandler);
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
