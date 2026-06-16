package frc.robot.lib.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.experimental.Accessors;

public class Toggle {
  private BooleanSupplier button;
  private boolean prevButton = false;

  @Accessors(fluent = true)
  @Getter
  private boolean toggled = false;

  public Toggle(BooleanSupplier button) {
    this.button = button;
  }

  public void periodic() {
    if (DriverStation.isTeleop()) {

      if (button.getAsBoolean() && !prevButton) {
        toggled = !toggled;
      }

      prevButton = button.getAsBoolean();
    }
  }

  public void reset() {
    toggled = false;
    prevButton = false;
  }
}
