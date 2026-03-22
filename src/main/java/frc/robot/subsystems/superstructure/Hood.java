package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.superstructure.LinearSubsystem;
import frc.robot.lib.superstructure.MotorConfig;
import org.littletonrobotics.junction.Logger;

public abstract class Hood extends LinearSubsystem {
  public static final Distance UPPER_LIMIT = Inches.of(3.3);
  public static final Distance LOWER_LIMIT = Inches.of(0.0);

  private static final double B = 4.8;
  private static final double C = 6.181717;
  private static final double ANGLE_OFFSET_RAD = 1.832;
  private static final double SCREW_OFFSET_INCH = 3.0;

  public Hood(MotorConfig motorConfig) {
    super(motorConfig);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        name + "/Angular Position (Deg)",
        leadScrewInchesToHoodAngle(Meters.of(io.getPosition().in(Rotations))).in(Degrees));

    if (goalPosition != null) {
      Logger.recordOutput(
          name + "/Goal Angular Position (Deg)",
          leadScrewInchesToHoodAngle(Meters.of(goalPosition.in(Rotations))).in(Degrees));
    }
  }

  @Override
  public void setPosition(Angle hoodAngle) {
    Distance goalDist = hoodAngleToLeadScrewInches(hoodAngle);
    Distance clampedGoalPosition =
        Meters.of(
            MathUtil.clamp(goalDist.in(Meters), LOWER_LIMIT.in(Meters), UPPER_LIMIT.in(Meters)));
    super.setPositionVoltage(clampedGoalPosition);
  }

  @Override
  public void setPosition(Distance goalDist) {
    Distance clampedGoalPosition =
        Meters.of(
            MathUtil.clamp(goalDist.in(Meters), LOWER_LIMIT.in(Meters), UPPER_LIMIT.in(Meters)));
    super.setPositionVoltage(clampedGoalPosition);
  }

  public Angle getHoodAngle() {
    return leadScrewInchesToHoodAngle(getLinearPosition());
  }

  // These equations are derived from the law of cosines.
  // Desmos is here: https://www.desmos.com/calculator/t72clpwl1j
  public static Distance hoodAngleToLeadScrewInches(Angle hoodAngle) {
    double x = hoodAngle.in(Radians);
    double L =
        Math.sqrt(B * B + C * C - 2.0 * B * C * Math.cos(ANGLE_OFFSET_RAD - x)) - SCREW_OFFSET_INCH;
    return Inches.of(L);
  }

  public static Angle leadScrewInchesToHoodAngle(Distance leadScrewInches) {
    double L = leadScrewInches.in(Inches);
    double cosArg = (B * B + C * C - Math.pow((L + SCREW_OFFSET_INCH), 2)) / (2.0 * B * C);
    double x = -Math.acos(cosArg) + ANGLE_OFFSET_RAD;
    return Radians.of(x);
  }
}
