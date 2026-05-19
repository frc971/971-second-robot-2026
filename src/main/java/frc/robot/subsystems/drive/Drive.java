package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
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

  private static final double ROTATION_DEADBAND = 0.2;

  public enum Mode {
    NONE,
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
    if (mode == Mode.NONE) {
      drivetrain.setRequest(freezeRequest);
    }
  }

  public void periodic() {
    updateMode();

    manual.setGoal(Manual.Goal.NONE);
    thetaLock.setGoal(ThetaLock.Goal.NONE);
    autoAlign.setGoal(AutoAlign.Goal.NONE);

    switch (mode) {
      case MANUAL -> manual.setGoal(Manual.Goal.ACTIVE);
      case AUTO_ALIGN -> autoAlign.setGoal(AutoAlign.Goal.ALIGN);
      case THETA_LOCK -> thetaLock.setGoal(ThetaLock.Goal.ACTIVE);
      default -> {}
    }

    manual.periodic();
    thetaLock.periodic();
    autoAlign.periodic();
  }

  private void updateMode() {
    if (DriverStation.isDisabled()) {
      setDriveMode(Mode.NONE);
    } else if (DriverStation.isTeleop() && mode == Mode.NONE) {
      setDriveMode(Mode.MANUAL);
    }
  }
}
