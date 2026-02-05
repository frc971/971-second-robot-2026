package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controllers {
  public static final CommandXboxController XBOX = new CommandXboxController(0);
  public static final Joystick BUTTONBOARD = new Joystick(1);

  public static final JoystickButton INTAKE_ROLLERS = new JoystickButton(BUTTONBOARD, 1);
  public static final JoystickButton INTAKE_PIVOT = new JoystickButton(BUTTONBOARD, 2);

  public static final JoystickButton MANUAL_BOTH = new JoystickButton(BUTTONBOARD, 2);
  public static final JoystickButton MANUAL_LEFT = new JoystickButton(BUTTONBOARD, 3);
  public static final JoystickButton MANUAL_RIGHT = new JoystickButton(BUTTONBOARD, 4);

  public static final JoystickButton SHOOT_BOTH = new JoystickButton(BUTTONBOARD, 5);
  public static final JoystickButton SHOOT_LEFT = new JoystickButton(BUTTONBOARD, 6);
  public static final JoystickButton SHOOT_RIGHT = new JoystickButton(BUTTONBOARD, 7);

  public static final JoystickButton TUNE_LEFT = new JoystickButton(BUTTONBOARD, 8);
  public static final JoystickButton TUNE_RIGHT = new JoystickButton(BUTTONBOARD, 9);
}
