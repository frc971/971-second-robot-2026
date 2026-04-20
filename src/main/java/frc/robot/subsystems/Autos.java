package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.List;
import java.util.Map;
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

  @Getter private final SendableChooser<AutoPathOption> chooser = new SendableChooser<>();

  private final FollowPath.Builder pathBuilder;

  public Autos(CommandSwerveDrivetrain drivetrain) {

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
    if (COMPOSED_AUTOS != null && COMPOSED_AUTOS.size() > 0) {

      for (String fullAutoName : COMPOSED_AUTOS.keySet()) {
        AutoPathOption normal = new AutoPathOption(fullAutoName, false);
        AutoPathOption mirrored = new AutoPathOption(fullAutoName, true);

        chooser.addOption(fullAutoName, normal);
        chooser.addOption(fullAutoName + " (Mirrored)", mirrored);
      }
    }
  }

  public Command getAutonomousCommand() {
    AutoPathOption selected = chooser.getSelected();

    if (selected == null) {
      return Commands.none();
    }

    String lookupName = selected.name.replace(" (Mirrored)", "");
    List<String> sequence = COMPOSED_AUTOS.get(lookupName);

    if (sequence != null) {
      Command auto = Commands.none();
      for (String name : sequence) {
        auto = auto.andThen(pathBuilder.build(new Path(name)));
      }
      return auto;
    }

    return pathBuilder.build(new Path(selected.name));
  }

  public Pose2d getAutonomousStartPose() {
    AutoPathOption selected = chooser.getSelected();

    if (selected == null) {
      return null;
    }

    Path path = new Path(selected.name);

    if (selected.mirrored) path.mirror();

    return path.getStartPose();
  }

  // IMPORTANT: all autos must be defined here
  // Map auto name (as displayed for user) to list of composed autos (IN ORDER)
  public static final Map<String, List<String>> COMPOSED_AUTOS =
      Map.of(
          "L_swipe", List.of("C_shoot", "L_swipe", "C_shoot", "L_swipe"),
          "Kev", List.of("Kev"));
}
