package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.*;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drive {

  private final CommandSwerveDrivetrain drivetrain;

  private final Manual manual;
  private final ThetaLock thetaLock;
  private final AutoAlign autoAlign;
  private final SwerveRequest.SwerveDriveBrake freezeRequest = new SwerveRequest.SwerveDriveBrake();

  public enum Mode {
    NONE,
    BRAKE,
    MANUAL,
    THETA_LOCK,
    AUTO_ALIGN;
  }

  @AutoLogOutput @Getter @Setter private Mode mode = Mode.MANUAL;

  public Drive(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;

    this.manual = new Manual(drivetrain);

    this.thetaLock = new ThetaLock(drivetrain, manual);
    this.autoAlign = new AutoAlign(drivetrain);
  }

  public void setDriveMode(Mode targetMode) {
    mode = targetMode;
  }

  public Command setDriveModeCommand(Mode targetMode) {
    return Commands.runOnce(() -> setDriveMode(targetMode));
  }

  public void periodic() {
    updateMode();

    manual.setGoal(Manual.Goal.NONE);
    thetaLock.setGoal(ThetaLock.Goal.NONE);
    autoAlign.setGoal(AutoAlign.Goal.NONE);

    switch (mode) {
      case BRAKE -> drivetrain.setRequest(freezeRequest);
      case MANUAL -> manual.setGoal(Manual.Goal.ACTIVE);
      case AUTO_ALIGN -> autoAlign.setGoal(AutoAlign.Goal.ALIGN);
      case THETA_LOCK -> thetaLock.setGoal(ThetaLock.Goal.ACTIVE);
      case NONE -> {}
    }

    manual.periodic();
    thetaLock.periodic();
    autoAlign.periodic();

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
    } else if (DriverStation.isTeleop() && (mode == Mode.NONE || mode == Mode.BRAKE)) {
      setDriveMode(Mode.MANUAL);
    }
  }
}
