package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A timed collection sweep toward a fixed field position. */
class UnetCollectSegment implements Segment {
  static final double UNET_SWEEP_SECONDS = 3.0;
  private static final Translation2d TARGET =
      new Translation2d(FlippingUtil.fieldSizeX / 2, FlippingUtil.fieldSizeY / 2);
  private final Subsystem drivetrain;
  private final Consumer<SwerveRequest> output;
  private final Supplier<Command> intakeDown;
  private final Supplier<Pose2d> pose;
  private final Supplier<Alliance> alliance;
  private final SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private long startedAt;

  UnetCollectSegment(CommandSwerveDrivetrain drivetrain, Supplier<Command> intakeDown) {
    this(
        drivetrain,
        drivetrain::setRequest,
        intakeDown,
        () -> drivetrain.getState().Pose,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  UnetCollectSegment(
      Subsystem drivetrain,
      Consumer<SwerveRequest> output,
      Supplier<Command> intakeDown,
      Supplier<Pose2d> pose,
      Supplier<Alliance> alliance) {
    this.pose = pose;
    this.alliance = alliance;
    this.drivetrain = drivetrain;
    this.output = output;
    this.intakeDown = intakeDown;
  }

  public Command getCommand(FollowPath.Builder builder) {
    return Commands.sequence(
        intakeDown.get(),
        new FunctionalCommand(
                this::initialize, this::execute, interrupted -> stop(), () -> false, drivetrain)
            .withTimeout(UNET_SWEEP_SECONDS));
  }

  private void initialize() {
    startedAt = NetworkTablesJNI.now();
    output.accept(brake);
    Logger.recordOutput("Auto/UNetCollect/Active", true);
    Logger.recordOutput("Auto/UNetCollect/DeadzoneEnabled", UnetDeadzone.ENABLED);
  }

  private void execute() {
    var currentPose = pose.get();
    var relative =
        TARGET.minus(currentPose.getTranslation()).rotateBy(currentPose.getRotation().unaryMinus());
    double distance = relative.getNorm();
    Logger.recordOutput(
        "Auto/UNetCollect/ElapsedSeconds", (NetworkTablesJNI.now() - startedAt) / 1e6);
    Logger.recordOutput("Auto/UNetCollect/TargetX", TARGET.getX());
    Logger.recordOutput("Auto/UNetCollect/TargetY", TARGET.getY());
    if (Double.isFinite(distance) && distance > 0.15) {
      var requested = relative.times(Math.min(1.5, distance * 2) / distance);
      var limited =
          UnetDeadzone.limit(currentPose, requested, alliance.get(), UnetDeadzone.ENABLED);
      Logger.recordOutput(
          "Auto/UNetCollect/RequestedVelocity", new double[] {requested.getX(), requested.getY()});
      output.accept(
          drive.withVelocityX(limited.getX()).withVelocityY(limited.getY()).withRotationalRate(0));
      logVelocity(limited.getX(), limited.getY(), 0);
    } else {
      output.accept(brake);
      logVelocity(0, 0, 0);
    }
  }

  private void logVelocity(double x, double y, double omega) {
    Logger.recordOutput("Auto/UNetCollect/VelocityX", x);
    Logger.recordOutput("Auto/UNetCollect/VelocityY", y);
    Logger.recordOutput("Auto/UNetCollect/OmegaRadiansPerSecond", omega);
  }

  private void stop() {
    output.accept(brake);
    logVelocity(0, 0, 0);
    Logger.recordOutput("Auto/UNetCollect/Active", false);
  }

  public Pose2d getStartPose(boolean mirrored) {
    return null;
  }

  public Path copy() {
    return new Path();
  }
}
