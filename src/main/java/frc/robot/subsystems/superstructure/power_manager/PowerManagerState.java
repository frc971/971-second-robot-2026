package frc.robot.subsystems.superstructure.power_manager;

import lombok.Builder;
import lombok.experimental.Delegate;

public enum PowerManagerState {
  DEFAULT(
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

  SHOOTING(
      LimitConfig.builder()
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
          .build()),

  MANUAL(
      LimitConfig.builder()
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
          .build()),

  SHUTTLING(
      LimitConfig.builder()
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
          .build()),

  SUPERCHARGED(
      LimitConfig.builder()
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
