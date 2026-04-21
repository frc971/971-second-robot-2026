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
import lombok.Getter;

public class Autos {

  /**
   * @param displayLabel what driverstation displays as the auto's name
   * @param pathNames what paths (in order) make up the auto
   */
  public record AutoRoutine(String displayLabel, List<String> pathNames) {
    public AutoRoutine {
      pathNames = List.copyOf(pathNames);
    }
  }

  private static class AutoPathOption {
    public final AutoRoutine routine;
    public final boolean mirrored;

    public AutoPathOption(AutoRoutine routine, boolean mirrored) {
      this.routine = routine;
      this.mirrored = mirrored;
    }

    @Override
    public String toString() {
      return routine.displayLabel() + (mirrored ? " (Mirrored)" : "");
    }
  }

  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  @Getter private final SendableChooser<AutoPathOption> chooser = new SendableChooser<>();

  private final FollowPath.Builder pathBuilderWithStartPoseReset;
  private final FollowPath.Builder pathBuilderContinuation;

  public Autos(CommandSwerveDrivetrain drivetrain) {

    chooser.setDefaultOption("None", null);

    // Build chooser FIRST so builder can reference it
    populateChooser();

    pathBuilderWithStartPoseReset = newPathBuilder(drivetrain).withPoseReset(drivetrain::resetPose);
    pathBuilderContinuation = newPathBuilder(drivetrain);

    SmartDashboard.putData("Auto Mode", chooser);
  }

  private FollowPath.Builder newPathBuilder(CommandSwerveDrivetrain drivetrain) {
    return new FollowPath.Builder(
            drivetrain,
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Speeds,
            speeds -> drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withShouldMirror(
            () -> {
              AutoPathOption selected = chooser.getSelected();
              return selected != null && selected.mirrored;
            })
        .withDefaultShouldFlip();
  }

  private void populateChooser() {
    for (AutoRoutine routine : AUTO_ROUTINES) {
      chooser.addOption(routine.displayLabel(), new AutoPathOption(routine, false));
      chooser.addOption(routine.displayLabel() + " (Mirrored)", new AutoPathOption(routine, true));
    }
  }

  public Command getAutonomousCommand() {
    AutoPathOption selected = chooser.getSelected();

    if (selected == null) {
      return Commands.none();
    }

    Command auto = Commands.none();
    List<String> segments = selected.routine.pathNames();
    for (int i = 0; i < segments.size(); i++) {
      FollowPath.Builder builder = i == 0 ? pathBuilderWithStartPoseReset : pathBuilderContinuation;
      auto = auto.andThen(builder.build(new Path(segments.get(i))));
    }
    return auto;
  }

  public Pose2d getAutonomousStartPose() {
    AutoPathOption selected = chooser.getSelected();

    if (selected == null) {
      return null;
    }

    List<String> segments = selected.routine.pathNames();
    if (segments.isEmpty()) {
      return null;
    }

    Path path = new Path(segments.get(0));

    if (selected.mirrored) path.mirror();

    return path.getStartPose();
  }

  // IMPORTANT: all autos must be defined here
  // list JSON names (without .json), in order
  public static final List<AutoRoutine> AUTO_ROUTINES =
      List.of(
          new AutoRoutine("Niko", List.of("S_Niko", "H_Niko_Normal", "F_Niko2nd", "H_Niko_Depot")),
          new AutoRoutine(
              "Niko No-depot", List.of("S_Niko", "H_Niko_Normal", "F_Niko2nd", "H_Niko_Normal")),
          new AutoRoutine("L_swipe", List.of("C_shoot", "L_swipe", "C_shoot", "L_swipe")),
          new AutoRoutine("Kev", List.of("kev")),
          new AutoRoutine(
              "Test Continuity",
              List.of("test_A", "test_B", "test_C", "test_D", "test_A", "test_B", "test_A")));
}
