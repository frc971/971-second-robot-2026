package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Autos {

  private record Routine(String name, boolean canMirror, String... paths) {}

  public record PreparedAuto(String name, Command command, Pose2d startPose) {}

  private record AutoOption(PreparedAuto blue, PreparedAuto red) {}

  private static final PreparedAuto NONE = new PreparedAuto("None", null, null);

  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private final SendableChooser<AutoOption> chooser = new SendableChooser<>();
  private final FollowPath.Builder firstPath;
  private final FollowPath.Builder nextPath;

  public Autos(CommandSwerveDrivetrain drivetrain) {
    firstPath = pathBuilder(drivetrain).withPoseReset(drivetrain::resetPose);
    nextPath = pathBuilder(drivetrain);
    chooser.setDefaultOption(NONE.name(), null);

    Map<String, Path> pathsByName = new HashMap<>();
    for (Routine routine : ROUTINES) {
      try {
        List<Path> paths =
            Arrays.stream(routine.paths())
                .map(name -> pathsByName.computeIfAbsent(name, Autos::loadPath))
                .toList();
        addOption(routine.name(), paths, false);
        if (routine.canMirror()) {
          addOption(routine.name() + " (Mirrored)", paths, true);
        }
      } catch (RuntimeException e) {
        DriverStation.reportError(
            "Could not prepare auto " + routine.name() + ": " + e.getMessage(), e.getStackTrace());
      }
    }
    SmartDashboard.putData("Auto Mode", chooser);
  }

  public PreparedAuto getSelectedAuto() {
    AutoOption option = chooser.getSelected();
    if (option == null) {
      return NONE;
    }
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? option.red()
        : option.blue();
  }

  private static Path loadPath(String name) {
    Path path = new Path(name);
    if (!path.isValid()) {
      throw new IllegalArgumentException("Invalid path " + name);
    }
    return path;
  }

  private void addOption(String name, List<Path> paths, boolean mirrored) {
    chooser.addOption(
        name,
        new AutoOption(
            buildAuto(name, paths, false, mirrored), buildAuto(name, paths, true, mirrored)));
  }

  private PreparedAuto buildAuto(String name, List<Path> paths, boolean red, boolean mirrored) {
    Command[] commands = new Command[paths.size()];
    Pose2d startPose = null;
    for (int i = 0; i < paths.size(); i++) {
      Path path = paths.get(i).copy();
      // Match BLine's transform order and keep the prepared commands reusable.
      if (red) {
        path.flip();
      }
      if (mirrored) {
        path.mirror();
      }
      if (i == 0) {
        startPose = path.getStartPose();
      }
      commands[i] = (i == 0 ? firstPath : nextPath).build(path);
    }
    return new PreparedAuto(name, Commands.sequence(commands), startPose);
  }

  private FollowPath.Builder pathBuilder(CommandSwerveDrivetrain drivetrain) {
    return new FollowPath.Builder(
        drivetrain,
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds,
        speeds -> drivetrain.setRequest(request.withSpeeds(speeds)),
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(3.0, 0.0, 0.0),
        new PIDController(2.0, 0.0, 0.0));
  }

  // Path file names in execution order. S = start, F = fuel, H = hub, D = depot.
  private static final List<Routine> ROUTINES =
      List.of(
          // Madtown Depot
          new Routine("Niko", false, "S_Normal", "H_Normal", "F_Normal", "D_Normal"),
          new Routine("Tamed Niko", false, "S_Normal", "H_Normal", "F_Normal_Tamed", "D_Normal"),
          new Routine("Short Niko", false, "S_Short", "H_Normal", "F_Short", "D_Normal"),
          new Routine(
              "Short Tamed Niko", false, "S_Short", "H_Normal", "F_Short_Tamed", "D_Normal"),

          // Madtown No Depot
          new Routine("James", true, "S_Normal", "H_Normal", "F_Normal", "H_Normal", "F_Normal"),
          new Routine(
              "Tamed James",
              true,
              "S_Normal",
              "H_Normal",
              "F_Normal_Tamed",
              "H_Normal",
              "F_Normal_Tamed"),
          new Routine("Short James", true, "S_Short", "H_Normal", "F_Short", "H_Normal"),
          new Routine(
              "Short Tamed James", true, "S_Short", "H_Normal", "F_Short_Tamed", "H_Normal"),

          // Supersteal
          new Routine(
              "SuperSteal Depot", false, "S_SuperSteal", "H_Normal", "F_SuperSteal", "D_Normal"),
          new Routine("SuperSteal", true, "S_SuperSteal", "H_Normal", "F_SuperSteal", "H_Normal"),

          // Middle Depot
          new Routine("BUM", false, "MiddleDepot2"));
}
