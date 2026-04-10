package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.superstructure.Edge;
import frc.robot.lib.superstructure.Toggle;

public class Controllers {
  public static final CommandXboxController TROY = new CommandXboxController(0);
  public static final CommandXboxController ANDRE = new CommandXboxController(1);

  // === Troy's Controls
  public static final Toggle INTAKE_PIVOT = new Toggle(TROY.leftBumper());
  public static final Trigger JUICE = TROY.a();
  public static final Edge INTAKE_PIVOT_EDGE = new Edge(TROY.leftBumper());
  public static final Trigger INTAKE_ROLLERS = TROY.leftTrigger();
  public static final Trigger UNJAM = ANDRE.rightStick();

  public static final Trigger SHOOT_REDUNDANCY = TROY.axisGreaterThan(3, 0.9);
  public static final Trigger INDEX = TROY.rightTrigger();

  public static final Trigger ODOMETRY_RESET = TROY.y();

  // === Andre's Controls
  public static final Trigger LEFT_SHUTTLE = ANDRE.leftBumper();
  public static final Trigger RIGHT_SHUTTLE = ANDRE.rightBumper();
  public static final Trigger SHOOT = ANDRE.axisGreaterThan(3, 0.9);

  public static final Trigger OUTTAKE = ANDRE.leftTrigger();

  public static final Toggle KILL_LEFT = new Toggle(ANDRE.x());
  public static final Toggle KILL_RIGHT = new Toggle(ANDRE.b());
  public static final Toggle MANUAL = new Toggle(ANDRE.y());
  public static final Trigger DISABLE_OTF = ANDRE.a();

  public static final Trigger SHUTTLE_LEFT = ANDRE.leftBumper();
  public static final Trigger SHUTTLE_RIGHT = ANDRE.rightBumper();

  // OTF Tuning (AndreTuner)
  public static final Trigger TUNE_LEFT = ANDRE.back();
  public static final Trigger TUNE_RIGHT = ANDRE.start();

  public static final Edge FLYWHEEL_UP = new Edge(ANDRE.povUp());
  public static final Edge FLYWHEEL_DOWN = new Edge(ANDRE.povDown());
  public static final Edge TURRET_LEFT = new Edge(ANDRE.povLeft());
  public static final Edge TURRET_RIGHT = new Edge(ANDRE.povRight());

  // MANUAL Shooter Setpoints
  public static final Trigger MANUAL_SHOOT_UP = ANDRE.axisLessThan(1, -0.85);
  public static final Trigger MANUAL_SHOOT_DOWN = ANDRE.axisGreaterThan(1, 0.85);
  public static final Trigger MANUAL_SHOOT_LEFT = ANDRE.axisLessThan(0, -0.85);
  public static final Trigger MANUAL_SHOOT_RIGHT = ANDRE.axisGreaterThan(0, 0.85);

  // MANUAL Shuttle Setpoints
  public static final Trigger MANUAL_SHUTTLE_UP = ANDRE.axisLessThan(5, -0.85);
  public static final Trigger MANUAL_SHUTTLE_DOWN = ANDRE.axisGreaterThan(5, 0.85);
  public static final Trigger MANUAL_SHUTTLE_LEFT = ANDRE.axisLessThan(4, -0.85);
  public static final Trigger MANUAL_SHUTTLE_RIGHT = ANDRE.axisGreaterThan(4, 0.85);
}
