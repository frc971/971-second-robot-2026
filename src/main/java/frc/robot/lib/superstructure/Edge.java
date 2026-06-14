package frc.robot.lib.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.experimental.Accessors;

public class Edge implements BooleanSupplier {
  private BooleanSupplier button;
  private boolean prevButton = false;

  @Accessors(fluent = true)
  @Getter
  private boolean rising = false;

  @Accessors(fluent = true)
  @Getter
  private boolean falling = false;

  public Edge(BooleanSupplier button) {
    this.button = button;
  }

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
