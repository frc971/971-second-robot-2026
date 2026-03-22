package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;

/** IMMUTABLE Stores position & velocity of an object Supports + & - operations */
public class ObjectState {
  private final Translation3d position; // in m
  private final Translation3d velocity; // in m/s

  public ObjectState(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  // for creating objectState from robotState
  public ObjectState(Pose2d pose, ChassisSpeeds chassisSpeeds) {
    this.position = new Translation3d(pose.getTranslation());
    this.velocity =
        new Translation3d(
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(pose.getRotation()));
  }

  public ObjectState(SwerveDriveState swerveState) {
    this(swerveState.Pose, swerveState.Speeds);
  }

  // Addition & Subtraction upgrade
  public ObjectState plus(ObjectState other) {
    return new ObjectState(position.plus(other.position), velocity.plus(other.velocity));
  }

  public ObjectState minus(ObjectState other) {
    return new ObjectState(position.minus(other.position), velocity.minus(other.velocity));
  }

  public Translation3d predictedPoseAfter(Time dt) {
    return position.plus(velocity.times(dt.in(Seconds))); // m + (m/s * s)
  }

  public ObjectState plus(Translation3d position, Translation3d velocity) {
    return new ObjectState(this.position.plus(position), this.velocity.plus(velocity));
  }

  public double speed() {
    return velocity.getNorm();
  }

  public Translation2d xyPos() {
    return position.toTranslation2d();
  }

  public Translation3d position() {
    return position;
  }

  public Translation3d velocity() {
    return velocity;
  }

  public ObjectState getFutureState(Time dt) {
    return new ObjectState(this.predictedPoseAfter(dt), velocity);
  }
}
