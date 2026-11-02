package frc.robot.lib.simulation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class FuelLayoutsTest {
  @Test
  void layoutsContainHundredsWithoutStartingOverlapsAndPushOutward() {
    for (int layout = 0; layout < 6; layout++) {
      var balls = FuelLayouts.create(layout);
      assertEquals(layout == 5 ? 480 : 360, balls.size());
      assertEquals(balls, FuelLayouts.create(layout));
      for (int i = 0; i < balls.size(); i++)
        for (int j = i + 1; j < balls.size(); j++)
          assertTrue(balls.get(i).getDistance(balls.get(j)) >= 0.152);
      if (layout == 0) continue;
      for (boolean red : new boolean[] {false, true})
        for (boolean mirror : new boolean[] {false, true}) {
          var p = new Translation2d(7.2, 6);
          if (mirror) p = new Translation2d(p.getX(), FlippingUtil.fieldSizeY - p.getY());
          if (red) p = FlippingUtil.flipFieldPosition(p);
          var v = FuelLayouts.outwardVelocity(layout, p, mirror, red);
          assertTrue((p.getX() - FlippingUtil.fieldSizeX / 2) * v.getX() > 0);
          assertTrue((p.getY() - FlippingUtil.fieldSizeY / 2) * v.getY() > 0);
        }
    }
  }

  @Test
  void impulseMovesExistingBallsAfterFiveSecondsAndSpatialHashResolvesNeighbors() {
    var sim = new FuelSim();
    sim.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        Pose2d::new,
        ChassisSpeeds::new);
    sim.configureOutwardPush(2, false, false);
    sim.spawnFuel(new Translation3d(7.2, 6, 0.075), new Translation3d());
    for (int i = 0; i < 250; i++) sim.stepSim();
    assertEquals(6, sim.getGroundFuelPositions().get(0).getY(), 1e-9);
    for (int i = 0; i < 50; i++) sim.stepSim();
    assertTrue(sim.getGroundFuelPositions().get(0).getY() > 6.3);
    sim.clearFuel();
    // An overlap spanning adjacent spatial-hash cells must still resolve.
    sim.spawnFuel(new Translation3d(7.19, 6, 0.075), new Translation3d());
    sim.spawnFuel(new Translation3d(7.31, 6, 0.075), new Translation3d());
    sim.stepSim();
    var balls = sim.getGroundFuelPositions();
    assertEquals(2, balls.size());
    assertTrue(balls.get(0).getDistance(balls.get(1)) >= 0.15 - 1e-9);
  }
}
