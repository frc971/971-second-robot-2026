package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.lib.JoystickValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manual {
  public enum Goal {
    NONE,
    ACTIVE
  };

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.ACTIVE;

  private final CommandSwerveDrivetrain drivetrain;

  private static final JoystickValues JOYSTICK_VALUES = new JoystickValues();
  // Control parameters
  private static final double SLEW_TRANSLATE_LIMIT = 100.0;
  private static final double SLEW_ROTATION_LIMIT = 100.0;
  private static final double TRANSLATION_EXP_CURVE = 2;
  private static final double ROTATION_EXP_CURVE = 2;
  private static final double TRANSLATION_DEADBAND = 0.05;
  private static final double ROTATION_DEADBAND = 0.1;

  // Rate limiters for smooth control
  private static final SlewRateLimiter X_LIMITER = new SlewRateLimiter(SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter Y_LIMITER = new SlewRateLimiter(SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter ROT_LIMITER = new SlewRateLimiter(SLEW_ROTATION_LIMIT);

  private static final double MAX_SPEED = 4.0; // in m/s

  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.90).in(RadiansPerSecond);

  private static final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * TRANSLATION_DEADBAND)
          .withRotationalDeadband(MAX_ANGULAR_RATE * ROTATION_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  public Manual(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void periodic() {

    JOYSTICK_VALUES
        .setValues(
            Controller.XBOX.getLeftY(), Controller.XBOX.getLeftX(), Controller.XBOX.getRightX())
        .exponentialCurve(TRANSLATION_EXP_CURVE, ROTATION_EXP_CURVE)
        .scale(
            -MAX_SPEED, // Negative max speed and angular rate since
            -MAX_ANGULAR_RATE) // controller inputs are reversed
        .slewRateLimit(X_LIMITER, Y_LIMITER, ROT_LIMITER);

    if (goal == Goal.NONE) {
      return;
    }
    drivetrain.setControl(
        drive
            .withVelocityX(JOYSTICK_VALUES.getX())
            .withVelocityY(JOYSTICK_VALUES.getY())
            .withRotationalRate(JOYSTICK_VALUES.getRot()));

    Logger.recordOutput("Drive/Manual/Joystick/X", Controller.XBOX.getLeftY());
    Logger.recordOutput("Drive/Manual/Joystick/Y", Controller.XBOX.getLeftX());
    Logger.recordOutput("Drive/Manual/Joystick/Rot", Controller.XBOX.getRightX());
  }

  public JoystickValues getValues() {
    return JOYSTICK_VALUES;
  }
}
