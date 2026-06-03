package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.superstructure.Edge;
import frc.robot.lib.superstructure.Toggle;

public class Controllers {
  public static final CommandXboxController DRIVER = new CommandXboxController(0);
  public static final CommandXboxController MANIPULATOR = new CommandXboxController(1);

  // === Driver's Controls
  public static final Toggle INTAKE_PIVOT = new Toggle(DRIVER.leftBumper());
  public static final Trigger JUICE = MANIPULATOR.a();
  public static final Edge INTAKE_PIVOT_EDGE = new Edge(DRIVER.leftBumper());
  public static final Trigger INTAKE_ROLLERS = DRIVER.leftTrigger();
  public static final Trigger UNJAM = MANIPULATOR.leftStick();

  public static final Trigger SHOOT_REDUNDANCY = DRIVER.axisGreaterThan(3, 0.9);
  public static final Trigger INDEX = DRIVER.rightTrigger();

  public static final Trigger ODOMETRY_RESET = DRIVER.y();

  // === Manipulator's Controls
  public static final Trigger LEFT_SHUTTLE = MANIPULATOR.leftBumper();
  public static final Trigger RIGHT_SHUTTLE = MANIPULATOR.rightBumper();
  public static final Trigger SHOOT = MANIPULATOR.axisGreaterThan(3, 0.9);
  public static final Trigger SHOOTING = SHOOT_REDUNDANCY.or(SHOOT);
  public static final Trigger SHUTTLING = LEFT_SHUTTLE.or(RIGHT_SHUTTLE).and(SHOOTING.negate());

  public static final Edge SHOOT_EDGE = new Edge(SHOOTING);
  public static final Edge SHUTTLE_EDGE = new Edge(SHUTTLING);

  public static final Trigger OUTTAKE = MANIPULATOR.leftTrigger();

  public static final Toggle MANUAL = new Toggle(MANIPULATOR.y());
  public static final Trigger DISABLE_OTF = MANIPULATOR.x();

  public static final Trigger SHUTTLE_LEFT = MANIPULATOR.leftBumper();
  public static final Trigger SHUTTLE_RIGHT = MANIPULATOR.rightBumper();

  // OTF Tuning
  public static final Trigger TUNE_LEFT = MANIPULATOR.back();
  public static final Trigger TUNE_RIGHT = MANIPULATOR.start();

  public static final Edge FLYWHEEL_UP = new Edge(MANIPULATOR.povUp());
  public static final Edge FLYWHEEL_DOWN = new Edge(MANIPULATOR.povDown());
  public static final Edge TURRET_LEFT = new Edge(MANIPULATOR.povLeft());
  public static final Edge TURRET_RIGHT = new Edge(MANIPULATOR.povRight());

  // MANUAL Shooter Setpoints
  public static final Trigger MANUAL_SHOOT_UP = MANIPULATOR.axisLessThan(1, -0.85);
  public static final Trigger MANUAL_SHOOT_DOWN = MANIPULATOR.axisGreaterThan(1, 0.85);
  public static final Trigger MANUAL_SHOOT_LEFT = MANIPULATOR.axisLessThan(0, -0.85);
  public static final Trigger MANUAL_SHOOT_RIGHT = MANIPULATOR.axisGreaterThan(0, 0.85);

  // MANUAL Shuttle Setpoints
  public static final Trigger MANUAL_SHUTTLE_UP = MANIPULATOR.axisLessThan(5, -0.85);
  public static final Trigger MANUAL_SHUTTLE_DOWN = MANIPULATOR.axisGreaterThan(5, 0.85);
  public static final Trigger MANUAL_SHUTTLE_LEFT = MANIPULATOR.axisLessThan(4, -0.85);
  public static final Trigger MANUAL_SHUTTLE_RIGHT = MANIPULATOR.axisGreaterThan(4, 0.85);
}
