package frc.robot.subsystems.superstructure;

import lombok.Builder;

public enum PowerManagerState {
  SHOOTING(
      Allocation.builder()
          .drivetrain(.35)
          .flywheel(.35)
          .groundRollers(.03)
          .groundPivot(.03)
          .hood(.04)
          .kicker(.06)
          .rollerFloor(.03)
          .b2(.03)
          .turret(.08)
          .build()),
  SHUTTLING(
      Allocation.builder()
          .drivetrain(.40)
          .flywheel(.28)
          .groundRollers(.03)
          .groundPivot(.03)
          .hood(.04)
          .kicker(.06)
          .rollerFloor(.03)
          .b2(.03)
          .turret(.10)
          .build()),
  DEFAULT(
      Allocation.builder()
          .drivetrain(.60)
          .flywheel(.10)
          .groundRollers(.06)
          .groundPivot(.05)
          .hood(.03)
          .kicker(.04)
          .rollerFloor(.04)
          .b2(.04)
          .turret(.04)
          .build());

  private final Allocation allocation;

  PowerManagerState(Allocation allocation) {
    this.allocation = allocation;
  }

  public Allocation allocation() {
    return allocation;
  }

  @Builder
  public record Allocation(
      double drivetrain,
      double flywheel,
      double groundRollers,
      double groundPivot,
      double hood,
      double kicker,
      double rollerFloor,
      double b2,
      double turret) {}
}
