package frc.robot.subsystems.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;
import lombok.Setter;

public class PausableAuto extends PathPlannerAuto {

  @Setter @Getter boolean paused = false;

  public PausableAuto(Command auto) {
    super(auto);
  }

  @Override
  public void execute() {
    if (!paused) {
      super.execute();
    }
  }
}
