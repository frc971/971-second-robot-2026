package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class UnetCollectSegmentTest {
  @Test
  void limitsFreshGoalsWithoutBlockingLateralMotionOrRotation() {
    HAL.initialize(500, 0);
    try (var nt = NetworkTableInstance.create();
        var publisher = nt.getTable("COS").getDoubleArrayTopic("BallVelocityGoal").publish()) {
      List<SwerveRequest> requests = new ArrayList<>();
      var segment =
          new UnetCollectSegment(
              new Subsystem() {},
              requests::add,
              nt,
              Commands::none,
              () ->
                  new Pose2d(
                      com.pathplanner.lib.util.FlippingUtil.fieldSizeX / 2,
                      1,
                      edu.wpi.first.math.geometry.Rotation2d.kZero),
              () -> Alliance.Blue);
      var command = segment.getCommand(null);
      command.initialize();
      command.execute();
      publisher.set(new double[] {2, 1, 0, 0, 0, 1, 0, 0, 0, 0.7});
      command.execute();
      var drive =
          assertInstanceOf(SwerveRequest.RobotCentric.class, requests.get(requests.size() - 1));
      assertEquals(0, drive.VelocityX, 1e-9);
      assertEquals(1, drive.VelocityY, 1e-9);
      assertEquals(0.7, drive.RotationalRate, 1e-9);
      command.end(true);
    }
  }

  @Test
  void followsFreshGoalsAndBrakesForInvalidStaleAndPreviousRunGoals() throws InterruptedException {
    HAL.initialize(500, 0);
    try (var nt = NetworkTableInstance.create();
        var publisher =
            nt.getTable("COS")
                .getDoubleArrayTopic("BallVelocityGoal")
                .publish(PubSubOption.keepDuplicates(true))) {
      List<SwerveRequest> requests = new ArrayList<>();
      var segment =
          new UnetCollectSegment(
              new Subsystem() {},
              requests::add,
              nt,
              Commands::none,
              Pose2d::new,
              () -> Alliance.Blue);
      var command = segment.getCommand(null);
      double[] goal = {6, 8, 2, 3, 0, 1, 0, 0, 0, -8};
      publisher.set(goal, NetworkTablesJNI.now() - 300_000);
      command.initialize();
      command.execute(); // Finish intake setup and initialize the sweep.
      command.execute();
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      publisher.set(goal);
      command.execute();
      var drive =
          assertInstanceOf(SwerveRequest.RobotCentric.class, requests.get(requests.size() - 1));
      assertEquals(3.5, Math.hypot(drive.VelocityX, drive.VelocityY), 1e-9);
      assertEquals(-0.8 * 2 * Math.PI, drive.RotationalRate, 1e-9);
      Thread.sleep(270); // Let the last valid publication exceed the freshness limit.
      command.execute();
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      publisher.set(new double[9]);
      command.execute();
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      publisher.set(goal);
      command.execute();
      assertInstanceOf(SwerveRequest.RobotCentric.class, requests.get(requests.size() - 1));
      command.end(true);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      command.initialize();
      command.execute();
      command.execute();
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      command.end(true);
    }
  }

  @Test
  void timesOutWithoutDetectionsAndHandsOffAfterIntake() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    try (var nt = NetworkTableInstance.create()) {
      List<String> events = new ArrayList<>();
      List<SwerveRequest> requests = new ArrayList<>();
      var segment =
          new UnetCollectSegment(
              new Subsystem() {},
              requests::add,
              nt,
              () -> Commands.runOnce(() -> events.add("intake")),
              Pose2d::new,
              () -> Alliance.Blue);
      var command = segment.getCommand(null).andThen(() -> events.add("return"));
      command.initialize();
      command.execute();
      assertEquals(List.of("intake"), events);
      SimHooks.stepTiming(2.98);
      command.execute();
      assertFalse(command.isFinished());
      SimHooks.stepTiming(0.04);
      command.execute();
      command.execute();
      assertEquals(List.of("intake", "return"), events);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
      command.end(false);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void replacesOnlySecondSweepAndKeepsMirrorSupport() {
    var original =
        Autos.AUTO_ROUTINES.stream()
            .filter(a -> a.displayLabel().equals("Dynamic real"))
            .findFirst()
            .orElseThrow();
    var unet =
        Autos.AUTO_ROUTINES.stream()
            .filter(a -> a.displayLabel().equals("Dynamic real UNet"))
            .findFirst()
            .orElseThrow();
    var expected = new ArrayList<>(original.pathNames());
    expected.set(2, "dynamic:unet");
    expected.add(2, "F_UNet_Entry");
    expected.add(3, "F_UNet_Turn");
    assertEquals(expected, unet.pathNames());
    assertTrue(unet.canMirror());
  }
}
