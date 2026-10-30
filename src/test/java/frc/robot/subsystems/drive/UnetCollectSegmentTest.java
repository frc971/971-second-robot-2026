package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class UnetCollectSegmentTest {
  @Test
  void drivesTowardFixedFieldTargetWithoutRotationAndStopsOnArrival() {
    HAL.initialize(500, 0);
    List<SwerveRequest> requests = new ArrayList<>();
    double x = FlippingUtil.fieldSizeX / 2, y = FlippingUtil.fieldSizeY / 2;
    Pose2d[] pose = {new Pose2d(x - 2, y - 2, edu.wpi.first.math.geometry.Rotation2d.kCCW_90deg)};
    var segment =
        new UnetCollectSegment(
            new Subsystem() {}, requests::add, Commands::none, () -> pose[0], () -> Alliance.Blue);
    var command = segment.getCommand(null);
    command.initialize();
    command.execute();
    command.execute();
    var drive =
        assertInstanceOf(SwerveRequest.RobotCentric.class, requests.get(requests.size() - 1));
    assertEquals(1.5 / Math.sqrt(2), drive.VelocityX, 1e-9);
    assertEquals(-1.5 / Math.sqrt(2), drive.VelocityY, 1e-9);
    assertEquals(0, drive.RotationalRate);
    pose[0] = new Pose2d(x, y - 0.2, edu.wpi.first.math.geometry.Rotation2d.kZero);
    command.execute();
    assertEquals(0.4, drive.VelocityY, 1e-9);
    pose[0] = new Pose2d(x, y, edu.wpi.first.math.geometry.Rotation2d.kZero);
    command.execute();
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, requests.get(requests.size() - 1));
    command.end(true);
  }

  @Test
  void timesOutAndHandsOffAfterIntake() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    try {
      List<String> events = new ArrayList<>();
      List<SwerveRequest> requests = new ArrayList<>();
      var segment =
          new UnetCollectSegment(
              new Subsystem() {},
              requests::add,
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
