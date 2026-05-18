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

  private final Superstructure superstructure;

  private final Manual manual;
  private final ThetaLock thetaLock;
  private final AutoAlign autoAlign;
  private final SwerveRequest.SwerveDriveBrake freezeRequest = new SwerveRequest.SwerveDriveBrake();

  private static final double ROTATION_DEADBAND = 0.2;

  public enum Mode {
    NONE(Manual.Goal.NONE, ThetaLock.Goal.NONE, AutoAlign.Goal.NONE),
    MANUAL(Manual.Goal.ACTIVE, ThetaLock.Goal.NONE, AutoAlign.Goal.NONE),
    THETA_LOCK(Manual.Goal.NONE, ThetaLock.Goal.ACTIVE, AutoAlign.Goal.NONE),
    AUTO_ALIGN(Manual.Goal.NONE, ThetaLock.Goal.NONE, AutoAlign.Goal.CLIMB);

    public final Manual.Goal manualGoal;
    public final ThetaLock.Goal thetaLockGoal;
    public final AutoAlign.Goal autoAlignGoal;

    Mode(Manual.Goal manualGoal, ThetaLock.Goal thetaLockGoal, AutoAlign.Goal autoAlignGoal) {
      this.manualGoal = manualGoal;
      this.thetaLockGoal = thetaLockGoal;
      this.autoAlignGoal = autoAlignGoal;
    }
  }

  @AutoLogOutput @Getter @Setter private Mode mode = Mode.MANUAL;

  public Drive(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {

    this.drivetrain = drivetrain;

    this.superstructure = superstructure;
    this.manual = new Manual(drivetrain);

    this.thetaLock = new ThetaLock(drivetrain, manual);
    this.autoAlign = new AutoAlign(drivetrain);
  }

  public void setDriveMode(Mode targetMode) {
    mode = targetMode;
    if (mode == Mode.NONE) {
      drivetrain.applyRequest(freezeRequest);
    }
  }

  public void periodic() {
    updateMode();

    manual.setGoal(mode.manualGoal);
    thetaLock.setGoal(mode.thetaLockGoal);
    autoAlign.setGoal(mode.autoAlignGoal);

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
