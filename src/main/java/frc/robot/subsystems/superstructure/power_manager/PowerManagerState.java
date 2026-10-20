package frc.robot.subsystems.superstructure.power_manager;

import lombok.Builder;
import lombok.experimental.Delegate;

public enum PowerManagerState {
  /*
     CURRENT LIMITS AT CHEZY:
      LimitConfig.builder()
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
          .build()),
  */

  AUTONOMOUS(
      LimitConfig.builder()
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
          .build()),
  DEFAULT(
      LimitConfig.builder()
          .flywheel(20)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(35)
          .drivetrainSteer(10)
          .build()),

  SHOOTING(
      LimitConfig.builder()
          .flywheel(50)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(15)
          .drivetrainSteer(10)
          .build()),

  MANUAL(
      LimitConfig.builder()
          .flywheel(50)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(35)
          .drivetrainSteer(10)
          .build()),

  SHUTTLING(
      LimitConfig.builder()
          .flywheel(40)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(35)
          .drivetrainSteer(10)
          .build()),

  SUPERCHARGED(
      LimitConfig.builder()
          .flywheel(20)
          .groundRollers(40)
          .groundPivot(25)
          .hood(25)
          .kicker(30)
          .rollerFloor(30)
          .b2(25)
          .turret(30)
          .drivetrain(50)
          .drivetrainSteer(20)
          .build());

  @Delegate private final LimitConfig limits;

  PowerManagerState(LimitConfig limits) {
    this.limits = limits;
  }

  @Builder
  public record LimitConfig(
      double flywheel,
      double groundRollers,
      double groundPivot,
      double hood,
      double kicker,
      double rollerFloor,
      double b2,
      double turret,
      double drivetrain,
      double drivetrainSteer) {}
}
