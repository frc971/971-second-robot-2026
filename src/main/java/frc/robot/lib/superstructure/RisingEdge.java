package frc.robot.lib.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.experimental.Accessors;

public class RisingEdge extends SubsystemBase {
  private Trigger button;
  private boolean prevButton = false;

  @Accessors(fluent = true)
  @Getter
  private boolean pressed = false;

  public RisingEdge(Trigger button) {
    this.button = button;
  }

  @Override
  public void periodic() {
    if (DriverStation.isTeleop()) {
      pressed = button.getAsBoolean() && !prevButton;
      prevButton = button.getAsBoolean();
    }
  }

  public void reset() {
    pressed = false;
    prevButton = false;
  }
}
