package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.superstructure.Toggle;

public class Controllers {
  public static final CommandXboxController TROY = new CommandXboxController(0);
  public static final CommandXboxController ANDRE = new CommandXboxController(1);

  // === Troy's Controls
  public static final Trigger INTAKE_FREEZE = TROY.leftBumper();
  public static final Trigger INTAKE = TROY.leftTrigger();
  public static final Trigger SHOOT_REDUNDANCY = TROY.rightTrigger();
  public static final Trigger CLIMB_EXTEND = TROY.y();
  public static final Trigger CLIMB_RETRACT = TROY.b(); // TODO: add drivetrain controls

  // === Software's Controls
  public static final Trigger RESET_TUNING = TROY.back();
  public static final Toggle TUNING = new Toggle(TROY.start());
  public static final Toggle TOGGLE_HOOD_FLYWHEEL = new Toggle(TROY.x());

  // TODO: do we need autoalign? This is the only button that is potentially free
  public static final Trigger AUTO_ALIGN = TROY.rightBumper();

  // score/save must be the same button, no space
  public static final Trigger SCORE = TROY.a();

  public static final Trigger UNDERSHOOT = TROY.povUp();
  public static final Trigger OVERSHOOT = TROY.povDown();
  public static final Trigger REVERT = TROY.povLeft();
  public static final Toggle TOGGLE_DRIVE = new Toggle(TROY.povRight());

  // === Andre's Controls
  public static final Trigger LEFT_SHUTTLE = ANDRE.leftBumper();
  public static final Trigger RIGHT_SHUTTLE = ANDRE.rightBumper();
  public static final Trigger SHOOT = ANDRE.rightTrigger();

  public static final Trigger OUTTAKE = ANDRE.leftTrigger();

  public static final Toggle KILL_LEFT = new Toggle(ANDRE.x());
  public static final Toggle KILL_RIGHT = new Toggle(ANDRE.b());
  public static final Toggle MANUAL = new Toggle(ANDRE.y());
  public static final Trigger REZERO = ANDRE.a();

  // OTF Tuning (AndreTuner)
  public static final Trigger FLYWHEEL_UP = ANDRE.povUp();
  public static final Trigger FLYWHEEL_DOWN = ANDRE.povDown();
  public static final Trigger TURRET_LEFT = ANDRE.povLeft();
  public static final Trigger TURRET_RIGHT = ANDRE.povRight();

  // TODO: get a feel for what axis threshold is the best to congrol with
  // MANUAL Shooter Setpoints
  public static final Trigger MANUAL_SHOOT_UP = ANDRE.axisGreaterThan(1, 0.85);
  public static final Trigger MANUAL_SHOOT_DOWN = ANDRE.axisLessThan(1, -0.85);
  public static final Trigger MANUAL_SHOOT_LEFT = ANDRE.axisGreaterThan(0, 0.85);
  public static final Trigger MANUAL_SHOOT_RIGHT = ANDRE.axisLessThan(0, -0.85);

  // MANUAL Shuttle Setpoints
  public static final Trigger MANUAL_SHUTTLE_UP = ANDRE.axisGreaterThan(1, 0.85);
  public static final Trigger MANUAL_SHUTTLE_DOWN = ANDRE.axisLessThan(1, -0.85);
  public static final Trigger MANUAL_SHUTTLE_LEFT = ANDRE.axisGreaterThan(0, 0.85);
  public static final Trigger MANUAL_SHUTTLE_RIGHT = ANDRE.axisLessThan(0, -0.85);
}
