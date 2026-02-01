package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controllers {
  public static final CommandXboxController XBOX = new CommandXboxController(0);
  public static final Joystick BUTTONBOARD = new Joystick(1);

  public static final JoystickButton INTAKE_ROLLERS = new JoystickButton(BUTTONBOARD, 1);
  public static final JoystickButton INTAKE_PIVOT = new JoystickButton(BUTTONBOARD, 2);
}
