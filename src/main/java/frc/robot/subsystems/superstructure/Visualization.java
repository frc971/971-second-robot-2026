package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class Visualization {
  public static final Transform3d robotToLeftTurret =
      new Transform3d(0.1624076 - 0.0144525, 0.224003 - 0.0144525, 0.2761615, Rotation3d.kZero);

  public static final Transform3d turretToLeftHood =
      new Transform3d(
          0.219786 - 0.1624076, 0.224003 - 0.224003, 0.445927 - 0.2761615, Rotation3d.kZero);

  public static final Transform3d robotToRightTurret =
      new Transform3d(0.1624076 - 0.0144525, -0.195097 - 0.0144525, 0.272987, Rotation3d.kZero);

  public static final Transform3d turretToRightHood =
      new Transform3d(
          0.219786 - 0.1624076, -0.195097 - -0.195097, 0.445927 - 0.2761615, Rotation3d.kZero);

  public static final Transform3d robotToGroundPivot =
      new Transform3d(-0.156033, 0.0, 0.132797, Rotation3d.kZero);

  private TurretLeft turretLeft;
  private HoodLeft hoodLeft;

  private TurretRight turretRight;
  private HoodRight hoodRight;

  private Climber climber;
  private GroundPivot groundPivot;

  public Visualization(
      TurretLeft turretLeft,
      TurretRight turretRight,
      HoodLeft hoodLeft,
      HoodRight hoodRight,
      Climber climber,
      GroundPivot groundPivot) {
    this.turretLeft = turretLeft;
    this.hoodLeft = hoodLeft;

    this.turretRight = turretRight;
    this.hoodRight = hoodRight;

    this.climber = climber;
    this.groundPivot = groundPivot;
  }

  public void periodic() {
    Transform3d turretLeftPose =
        robotToLeftTurret.plus(
            new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0.0, 0.0, turretLeft.getPosition().in(Radians))));

    Transform3d hoodLeftPose =
        turretLeftPose
            .plus(turretToLeftHood)
            .plus(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 1.30959265358 - hoodLeft.getHoodAngle().in(Radians), 0.0)));

    Transform3d turretRightPose =
        robotToRightTurret.plus(
            new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0.0, 0.0, turretRight.getPosition().in(Radians))));

    Transform3d hoodRightPose =
        turretRightPose
            .plus(turretToRightHood)
            .plus(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(
                        0.0, 1.30959265358 - hoodRight.getHoodAngle().in(Radians), 0.0)));

    Transform3d climberPose =
        new Transform3d(
            new Translation3d(0.0, 0.0, climber.getLinearPosition().in(Meters)), Rotation3d.kZero);

    Transform3d groundPivotPose =
        robotToGroundPivot.plus(
            new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0.0, groundPivot.getPosition().in(Radians), 0.0)));

    Logger.recordOutput(
        "Visualization/Components",
        turretLeftPose,
        hoodLeftPose,
        turretRightPose,
        hoodRightPose,
        climberPose,
        groundPivotPose);
  }
}
