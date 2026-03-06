package frc.robot.lib.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.experimental.Accessors;

public class Edge extends SubsystemBase implements BooleanSupplier {
  private Trigger button;
  private boolean prevButton = false;

  @Accessors(fluent = true)
  @Getter
  private boolean rising = false;

  @Accessors(fluent = true)
  @Getter
  private boolean falling = false;

  public Edge(Trigger button) {
    this.button = button;
  }

  @Override
  public void periodic() {
    boolean cur = button.getAsBoolean();
    if (DriverStation.isTeleop()) {
      rising = cur && !prevButton;
      falling = !cur && prevButton;
    }
    prevButton = cur;
  }

  @Override
  public boolean getAsBoolean() {
    return button.getAsBoolean();
  }

  public void reset() {
    rising = false;
    falling = false;
    prevButton = false;
  }
}
