package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.AutoAlign;
import org.junit.jupiter.api.*;

class AutoAlignTest {
  CommandSwerveDrivetrain drivetrain;
  AutoAlign handler;

  private static final Distance DISTANCE_TOLERANCE = Meters.of(0.02);
  private static final Angle ROTATION_TOLERANCE = Degrees.of(1.0);

  // Simulate the heading controller used in driveAtAngle
  PIDController headingController;

  @BeforeEach
  void setUp() {
    drivetrain = TunerConstants.createDrivetrain();
    handler = new AutoAlign(drivetrain);

    // Match the PID constants from driveAtAngle (5, 0, 0)
    headingController =
        new PIDController(AutoAlign.ROTATION_KP, AutoAlign.ROTATION_KI, AutoAlign.ROTATION_KD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  double normalizeDegrees(double theta) {
    return MathUtil.inputModulus(theta, -180, 180);
  }

  Pose2d simulate(Pose2d start, Pose2d target, double seconds, double dt) {
    Pose2d pose = start;

    for (double t = 0; t < seconds; t += dt) {
      SwerveRequest.FieldCentricFacingAngle swerveRequest = handler.computeAlignment(pose, target);

      // Extract translation velocities
      Translation2d delta =
          new Translation2d(swerveRequest.VelocityX, swerveRequest.VelocityY).times(dt);

      // Simulate the heading controller to get rotational rate
      double currentAngle = pose.getRotation().getRadians();
      double targetAngle = swerveRequest.TargetDirection.getRadians();
      double rotationalRate = headingController.calculate(currentAngle, targetAngle);
      Rotation2d dtheta = new Rotation2d(rotationalRate * dt);

      pose = new Pose2d(pose.getTranslation().plus(delta), pose.getRotation().plus(dtheta));
    }

    return pose;
  }

  void verifyReachesGoal(Pose2d start, Pose2d target, double seconds, double dt) {
    Pose2d result = simulate(start, target, seconds, dt);

    assertEquals(result.getX(), target.getX(), DISTANCE_TOLERANCE.in(Meters));
    assertEquals(result.getY(), target.getY(), DISTANCE_TOLERANCE.in(Meters));
    assertEquals(
        normalizeDegrees(result.getRotation().getDegrees()),
        normalizeDegrees(target.getRotation().getDegrees()),
        ROTATION_TOLERANCE.in(Degrees));
  }

  @Test
  void testTranslate() {
    verifyReachesGoal(
        new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(2, 0, new Rotation2d()),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));
  }

  @Test
  void testRotate() {
    verifyReachesGoal(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));
  }

  @Test
  void testTranslateAndRotate() {
    verifyReachesGoal(
        new Pose2d(1, 1, Rotation2d.fromDegrees(45)),
        new Pose2d(-1, 0, Rotation2d.fromDegrees(-90)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));
  }

  @Test
  void testComplexSequence() {
    verifyReachesGoal(
        new Pose2d(0.5, -1.2, Rotation2d.fromDegrees(45)),
        new Pose2d(-2.3, 0.7, Rotation2d.fromDegrees(-135)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(3.14, 2.71, Rotation2d.fromDegrees(179.5)),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.5)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(-0.5, 1.5, Rotation2d.fromDegrees(-90)),
        new Pose2d(2.2, -3.3, Rotation2d.fromDegrees(120)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(0.5)),
        5.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(-10.0, -10.0, Rotation2d.fromDegrees(-5000)),
        new Pose2d(10.0, 10.0, Rotation2d.fromDegrees(5000)),
        25.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(10.0, 10.0, Rotation2d.fromDegrees(5000)),
        new Pose2d(-10.0, -10.0, Rotation2d.fromDegrees(-5000)),
        25.0,
        Constants.UPDATE_PERIOD.in(Seconds));

    verifyReachesGoal(
        new Pose2d(5.5, -4.4, Rotation2d.fromDegrees(-45)),
        new Pose2d(-5.5, 4.4, Rotation2d.fromDegrees(135)),
        10.0,
        Constants.UPDATE_PERIOD.in(Seconds));
  }
}
