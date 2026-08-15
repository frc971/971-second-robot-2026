package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pathing {

  public enum Goal {
    NONE,
    ACTIVE,
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSubscriber vxSub;
  private final DoubleSubscriber vySub;
  private final BooleanSubscriber isDoneSub;
  private final StructPublisher<Pose2d> targetPub;
  private final NetworkTableEntry enabledEntry;

  @AutoLogOutput(key = "/pathing/enabled")
  private Goal goal = Goal.NONE;

  private double vx = 0;
  private double vy = 0;
  private double velocityTimestamp = 0;

  @AutoLogOutput(key = "/pathing/target")
  private Pose2d targetPose = new Pose2d();

  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public Pathing(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable pathing = inst.getTable("pathing");
    enabledEntry = pathing.getEntry("enabled");
    targetPub = pathing.getStructTopic("target", Pose2d.struct).publish();
    vxSub = pathing.getDoubleTopic("vx").subscribe(0.0);
    vySub = pathing.getDoubleTopic("vy").subscribe(0.0);
    isDoneSub = pathing.getBooleanTopic("isDone").subscribe(false);
    enabledEntry.setBoolean(false);
  }

  public void setGoal(Goal goal) {
    if (goal == Goal.NONE) {
      disable();
      return;
    }
    this.goal = goal;
    enabledEntry.setBoolean(true);
  }

  public void setTargetPose(Pose2d pose) {
    targetPose = pose;
    targetPub.set(pose);
    setGoal(Goal.ACTIVE);
  }

  public void disable() {
    goal = Goal.NONE;
    enabledEntry.setBoolean(false);
    vx = 0;
    vy = 0;
    drivetrain.setRequest(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  public void periodic() {
    boolean ntEnabled = enabledEntry.getBoolean(false);
    if (!ntEnabled || goal != Goal.ACTIVE) {
      if (goal == Goal.ACTIVE) {
        disable();
      }
      return;
    }
    updateVelocityFromNetworkTables();
    Pose2d pose = drivetrain.getState().Pose;
    ChassisSpeeds field = new ChassisSpeeds(vx, vy, 0);
    ChassisSpeeds robot = ChassisSpeeds.fromFieldRelativeSpeeds(field, pose.getRotation());
    drivetrain.setRequest(request.withSpeeds(robot));
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public boolean isPathComplete() {
    return isDoneSub.get();
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
    velocityTimestamp = Timer.getFPGATimestamp();
    vx = clamp(vxSub.get());
    vy = clamp(vySub.get());
  }

  private double clamp(double value) {
    return Math.max(
        -Drive.MAX_SPEED_METERS_PER_SECOND, Math.min(Drive.MAX_SPEED_METERS_PER_SECOND, value));
  }
}
