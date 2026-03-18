package frc.robot.lib.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.experimental.Accessors;

public class Toggle extends SubsystemBase {
  private Trigger button;
  private boolean prevButton = false;

  @Accessors(fluent = true)
  @Getter
  private boolean toggled = false;

  public Toggle(Trigger button) {
    this.button = button;
  }

  @Override
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
