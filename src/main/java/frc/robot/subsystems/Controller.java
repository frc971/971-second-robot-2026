package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
  public static final CommandXboxController XBOX = new CommandXboxController(0);

  public static final Trigger AUTO_ALIGN = XBOX.button(1);
  public static final Trigger GROUND_ROLLERS = XBOX.button(2);
  public static final Trigger INDEXER_BUTTON = XBOX.button(4);
  public static final Trigger SHOOT_BUTTON = XBOX.button(5);
  public static final Trigger TUNE_BUTTON = XBOX.button(6);
}
