package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Pathing {

  public enum Goal {
    NONE,
    ACTIVE
  }

  private final CommandSwerveDrivetrain drivetrain;

  private final NetworkTableInstance inst;

  private final DoubleArraySubscriber velocitySub;

  private final NetworkTableEntry enabledEntry;

  @AutoLogOutput(key = "Pathing/Goal")
  private Goal goal = Goal.NONE;

  private double vx = 0;
  private double vy = 0;
  private double velocityTimestamp = 0;

  @AutoLogOutput(key = "Pathing/TargetPose")
  private Pose2d targetPose = new Pose2d();

  private static final double MAX_AGE_SECONDS = 0.2;
  private static final double PATH_COMPLETE_X_TOLERANCE_METERS = 0.05;
  private static final double PATH_COMPLETE_Y_TOLERANCE_METERS = 0.05;

  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public Pathing(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;

    inst = NetworkTableInstance.getDefault();

    NetworkTable pathing = inst.getTable("Pathing");

    NetworkTable orin = inst.getTable("Orin").getSubTable("OTFVelocity");

    enabledEntry = pathing.getEntry("Enabled");

    velocitySub =
        orin.getDoubleArrayTopic("Velocity")
            .subscribe(new double[] {0, 0, 0}, PubSubOption.keepDuplicates(true));
  }

  public void setGoal(Goal goal) {
    if (goal == Goal.NONE) {
      disable();
      return;
    }

    this.goal = goal;
  }

  public void setTargetPose(Pose2d pose) {
    targetPose = pose;
    goal = Goal.ACTIVE;
  }

  public void disable() {
    goal = Goal.NONE;
    vx = 0;
    vy = 0;

    drivetrain.setRequest(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  public void periodic() {

    Pose2d pose = drivetrain.getState().Pose;

    boolean ntEnabled = enabledEntry.getBoolean(false);
    if (!ntEnabled || goal != Goal.ACTIVE) {
      if (goal == Goal.ACTIVE) {
        disable();
      }
      return;
    }

    updateVelocityFromNetworkTables();

    ChassisSpeeds field = new ChassisSpeeds(vx, vy, 0);

    ChassisSpeeds robot = ChassisSpeeds.fromFieldRelativeSpeeds(field, pose.getRotation());

    drivetrain.setRequest(request.withSpeeds(robot));
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public boolean isPathComplete() {
    Pose2d currentPose = drivetrain.getState().Pose;
    return Math.abs(targetPose.getX() - currentPose.getX()) < PATH_COMPLETE_X_TOLERANCE_METERS
        && Math.abs(targetPose.getY() - currentPose.getY()) < PATH_COMPLETE_Y_TOLERANCE_METERS;
  }

  @AutoLogOutput(key = "Pathing/CurrentPose")
  public Pose2d getCurrentPose() {
    return drivetrain.getState().Pose;
  }

  @AutoLogOutput(key = "Pathing/Velocity")
  public double[] getVelocity() {
    return new double[] {vx, vy, velocityTimestamp};
  }

  private void updateVelocityFromNetworkTables() {
    double timestamp = getNetworkTablesTimestampSeconds();
    velocityTimestamp = timestamp;

    double[] velocity = velocitySub.get();
    if (velocity == null || velocity.length < 3) {
      stop();
      return;
    }

    double age = timestamp - velocity[2];
    if (age < 0 || age >= MAX_AGE_SECONDS) {
      stop();
      return;
    }

    vx = clamp(velocity[0]);
    vy = clamp(velocity[1]);
  }

  private void stop() {
    vx = 0;
    vy = 0;
  }

  private double clamp(double v) {
    return Math.max(
        -Drive.MAX_SPEED_METERS_PER_SECOND, Math.min(Drive.MAX_SPEED_METERS_PER_SECOND, v));
  }

  private double getNetworkTablesTimestampSeconds() {
    long offsetUs = inst.getServerTimeOffset().orElse(0L);
    return Timer.getFPGATimestamp() + (offsetUs / 1_000_000.0);
  }
}
