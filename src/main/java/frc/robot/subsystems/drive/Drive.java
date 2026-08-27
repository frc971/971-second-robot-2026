package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drive {
  public static final double MAX_SPEED_METERS_PER_SECOND = 3.5;

  private final CommandSwerveDrivetrain drivetrain;
  private final Telemetry telemetry = new Telemetry(MAX_SPEED_METERS_PER_SECOND);

  private final Manual manual;
  private final ThetaLock thetaLock;
  private final AutoAlign autoAlign;
  private final Pathing pathing;
  private final SwerveRequest.SwerveDriveBrake freezeRequest = new SwerveRequest.SwerveDriveBrake();

  public enum Mode {
    NONE,
    BRAKE,
    MANUAL,
    THETA_LOCK,
    AUTO_ALIGN,
    PATHING;
  }

  @AutoLogOutput @Getter @Setter private Mode mode = Mode.MANUAL;

  public Drive(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    drivetrain.registerTelemetry(telemetry::telemeterize);

    this.manual = new Manual(drivetrain);
    this.pathing = new Pathing(drivetrain);

    this.thetaLock = new ThetaLock(drivetrain, manual);
    this.autoAlign = new AutoAlign(drivetrain);
  }

  public void setDriveMode(Mode targetMode) {
    mode = targetMode;
  }

  public Command setDriveModeCommand(Mode targetMode) {
    return Commands.runOnce(() -> setDriveMode(targetMode));
  }

  public void setPathingTargetPose(Pose2d targetPose) {
    pathing.setTargetPose(targetPose);
  }

  public void setAutoStartPose(Pose2d pose) {
    telemetry.setAutoStartPose(pose);
  }

  public void periodic() {
    updateMode();

    manual.setGoal(Manual.Goal.NONE);
    thetaLock.setGoal(ThetaLock.Goal.NONE);
    autoAlign.setGoal(AutoAlign.Goal.NONE);
    if (mode != Mode.PATHING) pathing.setGoal(Pathing.Goal.NONE);

    switch (mode) {
      case BRAKE -> drivetrain.setRequest(freezeRequest);
      case MANUAL -> manual.setGoal(Manual.Goal.ACTIVE);
      case AUTO_ALIGN -> autoAlign.setGoal(AutoAlign.Goal.ALIGN);
      case THETA_LOCK -> thetaLock.setGoal(ThetaLock.Goal.ACTIVE);
      case PATHING -> pathing.setGoal(Pathing.Goal.ACTIVE);
      case NONE -> {}
    }

    manual.periodic();
    thetaLock.periodic();
    autoAlign.periodic();
    pathing.periodic();

    drivetrain.periodic();
  }

  private void updateMode() {
    if (DriverStation.isDisabled()) {
      setDriveMode(Mode.BRAKE);
    } else if (DriverStation.isAutonomous()
        && mode == Mode.AUTO_ALIGN
        && autoAlign.getGoal() == AutoAlign.Goal.ALIGN
        && autoAlign.isAligned()) {
      setDriveMode(Mode.NONE);
    } else if (DriverStation.isTeleop() && mode != Mode.PATHING) {
      setDriveMode(Mode.MANUAL);
    }

    if (DriverStation.isTeleopEnabled()) {
      // TODO: add teleop specific triggered logic
    }
  }
}
