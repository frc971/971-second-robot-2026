package frc.robot.lib.simulation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

class CosBallPathingSimTest {
  @Test
  void producesBoundedRobotRelativeGoalsAndHandlesMissingOrReachedTargets() {
    var target = new Translation2d(3, 0);
    var goal = CosBallPathingSim.goal(new Pose2d(0, 0, Rotation2d.kCCW_90deg), target, true);
    assertEquals(0, goal[0], 1e-9);
    assertEquals(-1.5, goal[1], 1e-9);
    assertEquals(1, goal[5]);
    assertTrue(Math.abs(goal[9]) <= 2);
    assertArrayEquals(new double[10], CosBallPathingSim.goal(new Pose2d(), target, false));
    var stopped = CosBallPathingSim.goal(new Pose2d(target, Rotation2d.kZero), target, true);
    assertEquals(0, stopped[0]);
    assertEquals(0, stopped[1]);
    assertEquals(0, stopped[9]);
  }

  @Test
  void publishesMovingClusterTargetsAndSupportsDetectionLossAndDisable() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    Pose2d[] robot = {new Pose2d(3.57, 7.44, Rotation2d.kZero)};
    try (var nt = NetworkTableInstance.create();
        var sim = new CosBallPathingSim(() -> robot[0], nt);
        var subscriber =
            nt.getTable("COS")
                .getDoubleArrayTopic("BallVelocityGoal")
                .subscribe(new double[0], PubSubOption.keepDuplicates(true))) {
      sim.reset(false);
      // Integrate the robot-relative goal to demonstrate traversal of the three target columns.
      for (int i = 0; i < 300; i++) {
        sim.periodic();
        var goal = subscriber.get();
        var delta =
            new Translation2d(goal[0], goal[1]).rotateBy(robot[0].getRotation()).times(0.02);
        robot[0] =
            new Pose2d(
                robot[0].getTranslation().plus(delta),
                robot[0].getRotation().plus(new Rotation2d(goal[9] * 0.02)));
      }
      assertTrue(robot[0].getX() > FlippingUtil.fieldSizeX / 2);
      assertEquals(FlippingUtil.fieldSizeY / 2, robot[0].getY(), 0.2);
      nt.getTable("Sim/COS").getEntry("Detected").setBoolean(false);
      sim.periodic();
      assertEquals(0, subscriber.get()[5]);
      subscriber.readQueue();
      nt.getTable("Sim/COS").getEntry("Enabled").setBoolean(false);
      sim.periodic();
      assertEquals(0, subscriber.readQueue().length);
      nt.getTable("Sim/COS").getEntry("Enabled").setBoolean(true);
      nt.getTable("Sim/COS").getEntry("Detected").setBoolean(true);
      sim.reset(true);
      sim.periodic();
      var relative = new Translation2d(subscriber.get()[2], subscriber.get()[3]);
      var fieldTarget = robot[0].getTranslation().plus(relative.rotateBy(robot[0].getRotation()));
      assertEquals(FlippingUtil.fieldSizeY - FlippingUtil.fieldSizeY / 2, fieldTarget.getY(), 1e-9);
      robot[0] = new Pose2d();
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.notifyNewData();
      sim.reset(true);
      sim.periodic();
      relative = new Translation2d(subscriber.get()[2], subscriber.get()[3]);
      fieldTarget = robot[0].getTranslation().plus(relative.rotateBy(robot[0].getRotation()));
      var expected =
          FlippingUtil.flipFieldPosition(
              new Translation2d(
                  FlippingUtil.fieldSizeX / 2 - 0.4,
                  FlippingUtil.fieldSizeY - FlippingUtil.fieldSizeY / 2));
      assertEquals(expected.getX(), fieldTarget.getX(), 1e-9);
      assertEquals(expected.getY(), fieldTarget.getY(), 1e-9);
    } finally {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }
  }
}
