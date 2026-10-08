package frc.robot.subsystems.superstructure.power_manager;

import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
@Builder
@Getter
public class PowerManagerState {
  public static final PowerManagerState DEFAULT =
      builder()
          .flywheel(40)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(27)
          .drivetrainSteer(10)
          .build();

  public static final PowerManagerState SHOOTING =
      builder()
          .flywheel(50)
          .groundRollers(20)
          .groundPivot(15)
          .hood(40)
          .kicker(20)
          .rollerFloor(20)
          .b2(25)
          .turret(30)
          .drivetrain(20)
          .drivetrainSteer(10)
          .build();

  public static final PowerManagerState MANUAL =
      builder()
          .flywheel(50)
          .groundRollers(20)
          .groundPivot(15)
          .hood(40)
          .kicker(20)
          .rollerFloor(20)
          .b2(25)
          .turret(30)
          .drivetrain(20)
          .drivetrainSteer(10)
          .build();

  public static final PowerManagerState SHUTTLING =
      builder()
          .flywheel(60)
          .groundRollers(20)
          .groundPivot(15)
          .hood(40)
          .kicker(15)
          .rollerFloor(15)
          .b2(25)
          .turret(30)
          .drivetrain(27)
          .drivetrainSteer(10)
          .build();

  public static final PowerManagerState SUPERCHARGED =
      builder()
          .flywheel(50)
          .groundRollers(25)
          .groundPivot(25)
          .hood(40)
          .kicker(10)
          .rollerFloor(10)
          .b2(25)
          .turret(30)
          .drivetrain(27)
          .drivetrainSteer(10)
          .build();

  private final double flywheel;
  private final double groundRollers;
  private final double groundPivot;
  private final double hood;
  private final double kicker;
  private final double rollerFloor;
  private final double b2;
  private final double turret;
  private final double drivetrain;
  private final double drivetrainSteer;
}
