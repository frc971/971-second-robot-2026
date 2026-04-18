package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import lombok.Getter;

public class Autos {
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  @Getter private final SendableChooser<String> chooser;
  private final FollowPath.Builder pathBuilder;

  public Autos(CommandSwerveDrivetrain drivetrain) {
    pathBuilder =
        new FollowPath.Builder(
                drivetrain, // Subsystem requirement
                () -> drivetrain.getState().Pose, // Supplier of current robot pose
                () -> drivetrain.getState().Speeds, // Supplier<ChassisSpeeds> (robot-relative)
                (speeds) -> drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)),
                new PIDController(5.0, 0.0, 0.0), // translation — minimizes remaining distance
                new PIDController(3.0, 0.0, 0.0), // rotation    — minimizes heading error
                new PIDController(2.0, 0.0, 0.0) // cross-track — minimizes perpendicular deviation
                )
            .withDefaultShouldFlip() // auto-flip when on the red alliance
            .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("example_a", "example_a");
    chooser.addOption("example_b", "example_b");

    SmartDashboard.putData("Auto Mode", chooser);
  }

  public Command getAutonomousCommand() {
    String selectedAuto = chooser.getSelected();

    if (selectedAuto == null) {
      return Commands.none();
    }

    return pathBuilder.build(new Path(selectedAuto));
  }
}
