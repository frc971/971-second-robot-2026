package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;

/** IMMUTABLE Stores position & velocity of an object Supports + & - operations */
public class ObjectState {
  private final Translation2d position; // in m
  private final Translation2d velocity; // in m/s

  public ObjectState(Translation2d position, Translation2d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  public ObjectState(Pose2d pose, ChassisSpeeds chassisSpeeds) {
    this.position = pose.getTranslation();
    this.velocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(pose.getRotation());
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

  public Translation2d predictedPoseAfter(Time dt) {
    return position.plus(velocity.times(dt.in(Seconds))); // m + (m/s * s)
  }

  public ObjectState plus(Translation2d position, Translation2d velocity) {
    return new ObjectState(this.position.plus(position), this.velocity.plus(velocity));
  }

  public double speed() {
    return velocity.getNorm();
  }

  public Translation2d position() {
    return position;
  }

  public Translation2d velocity() {
    return velocity;
  }

  public ObjectState getFutureState(Time dt) {
    return new ObjectState(this.predictedPoseAfter(dt), velocity);
  }
}
