package frc.robot.lib.simulation;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/** Dense starting fuel and reproducible outward disturbances before the second collection pass. */
public final class FuelLayouts {
  public static final double PUSH_AT_SECONDS = 5.0;
  public static final String[] NAMES = {
    "Dense control",
    "Mild outward",
    "Wide outward",
    "Longitudinal outward",
    "Upper biased outward",
    "Scattered outward",
    "Empty"
  };

  public static List<Translation2d> create(int layout) {
    var balls = new ArrayList<Translation2d>();
    if (layout == 6) return List.of();
    // 360 non-overlapping balls, covering both halves of the neutral-zone center pile.
    for (int x = 0; x < 12; x++)
      for (int y = 0; y < 30; y++)
        balls.add(
            new Translation2d(
                FlippingUtil.fieldSizeX / 2 + (x - 5.5) * 0.154,
                FlippingUtil.fieldSizeY / 2 + (y - 14.5) * 0.154));
    if (layout == 5) {
      var random = new Random(971);
      for (int i = 0; i < 120; i++) {
        while (true) {
          var p =
              new Translation2d(6.05 + random.nextDouble() * 4.4, 0.9 + random.nextDouble() * 6.2);
          if (balls.stream().noneMatch(q -> q.getDistance(p) < 0.152)) {
            balls.add(p);
            break;
          }
        }
      }
    }
    return List.copyOf(balls);
  }

  public static Translation2d outwardVelocity(
      int layout, Translation2d position, boolean mirrored, boolean red) {
    if (layout <= 0 || layout == 6) return new Translation2d();
    var p = red ? FlippingUtil.flipFieldPosition(position) : position;
    if (mirrored) p = new Translation2d(p.getX(), FlippingUtil.fieldSizeY - p.getY());
    double dx = p.getX() - FlippingUtil.fieldSizeX / 2;
    double dy = p.getY() - FlippingUtil.fieldSizeY / 2;
    double ySpeed =
        switch (layout) {
          case 1 -> 0.18;
          case 2 -> 0.40;
          case 3 -> 0.12;
          case 4 -> dy > 0 ? 0.48 : 0.12;
          default -> 0.20 + 0.25 * (0.5 + 0.5 * Math.sin(p.getX() * 17 + p.getY() * 31));
        };
    double xSpeed = layout == 3 ? 0.24 : 0.04;
    var velocity = new Translation2d(Math.copySign(xSpeed, dx), Math.copySign(ySpeed, dy));
    if (mirrored) velocity = new Translation2d(velocity.getX(), -velocity.getY());
    return red ? velocity.unaryMinus() : velocity;
  }

  private FuelLayouts() {}
}
