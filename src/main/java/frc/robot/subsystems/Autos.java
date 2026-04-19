package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.io.File;
import lombok.Getter;

public class Autos {

  private static class AutoPathOption {
    public final String name;
    public final boolean mirrored;

    public AutoPathOption(String name, boolean mirrored) {
      this.name = name;
      this.mirrored = mirrored;
    }

    @Override
    public String toString() {
      return name + (mirrored ? " (Mirrored)" : "");
    }
  }

  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  @Getter private final SendableChooser<AutoPathOption> chooser;
  private final FollowPath.Builder pathBuilder;

  public Autos(CommandSwerveDrivetrain drivetrain) {

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("bumblebee", new AutoPathOption("bumblebee", false));

    // Build chooser FIRST so builder can reference it
    populateChooser();

    pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                speeds -> drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            // Custom mirror (left/right)
            .withShouldMirror(
                () -> {
                  AutoPathOption selected = chooser.getSelected();
                  return selected != null && selected.mirrored;
                })
            // Optional: keep alliance flip (remove if undesired)
            .withDefaultShouldFlip()
            .withPoseReset(drivetrain::resetPose);

    SmartDashboard.putData("Auto Mode", chooser);
  }

  private void populateChooser() {
    File pathsDir = new File(Filesystem.getDeployDirectory(), "autos/paths");
    File[] files = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));

    if (files != null && files.length > 0) {
      for (File file : files) {
        String baseName = file.getName().replace(".json", "");

        AutoPathOption normal = new AutoPathOption(baseName, false);
        AutoPathOption mirrored = new AutoPathOption(baseName, true);

        chooser.addOption(baseName, normal);
        chooser.addOption(baseName + " (Mirrored)", mirrored);
      }
    }
  }

  public Command getAutonomousCommand() {
    AutoPathOption selected = chooser.getSelected();

    if (selected == null) {
      return Commands.none();
    }

    return pathBuilder.build(new Path(selected.name));
  }
}
