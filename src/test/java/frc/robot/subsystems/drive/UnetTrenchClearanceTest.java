package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.SimSwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.simulation.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.*;
import org.ironmaple.simulation.drivesims.configs.*;
import org.junit.jupiter.api.Test;

/**
 * Runs the production path/UNet commands on MapleSim motors and dyn4j contacts (without CTRE
 * threads).
 */
class UnetTrenchClearanceTest {
  @Test
  void completeAutoHasZeroPhysicsContactsOnBothAlliancesAndMirrors() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    FollowPath.registerEventTrigger("intakeDown", () -> {});
    FollowPath.registerEventTrigger("shoot", () -> {});
    FollowPath.registerEventTrigger("neutral", () -> {});
    try {
      for (boolean red : new boolean[] {false, true}) {
        DriverStationSim.setAllianceStationId(
            red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        for (boolean mirror : new boolean[] {false, true})
          for (boolean detected : new boolean[] {false, true}) runAuto(red, mirror, detected);
      }
    } finally {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }

  @Test
  void contactCounterDetectsAnActualWallContact() {
    HAL.initialize(500, 0);
    var body =
        new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default(), new Pose2d(0.1, 4, Rotation2d.kZero));
    var arena = new CollisionTrackingArena(body);
    SimulatedArena.overrideInstance(arena);
    try {
      arena.simulationPeriodic();
      assertTrue(
          arena.getFieldContacts() > 0,
          "The collision listener must detect penetration of the field wall");
    } finally {
      arena.shutDown();
    }
  }

  private record Result(
      int collected, int at20, double seconds, int contacts, double shootingAt, int firstPass) {}

  @Test
  void benchmarkMatchedBallLayouts() throws Exception {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    var csv =
        new StringBuilder(
            "layout,red,mirror,auto,collected,at20,seconds,contacts,shootingAt,firstPass,secondPassAt20,balls\n");
    int jamesTotal = 0, unetTotal = 0, blindTotal = 0, wins = 0;
    int jamesSecond = 0, unetSecond = 0;
    try {
      for (int layout = 0; layout < FuelLayouts.NAMES.length; layout++) {
        var balls = FuelLayouts.create(layout);
        int jamesLayout = 0, unetLayout = 0;
        for (boolean red : new boolean[] {false, true})
          for (boolean mirror : new boolean[] {false, true}) {
            DriverStationSim.setAllianceStationId(
                red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();
            for (String auto : new String[] {"James", "Dynamic real UNet", "UNet blind"}) {
              var result =
                  runRoutine(
                      red,
                      mirror,
                      !auto.equals("UNet blind"),
                      auto.equals("James") ? auto : "Dynamic real UNet",
                      balls,
                      layout);
              csv.append(
                  String.format(
                      java.util.Locale.ROOT,
                      "%d,%s,%s,%s,%d,%d,%.2f,%d,%.2f,%d,%d,%d%n",
                      layout,
                      red,
                      mirror,
                      auto,
                      result.collected,
                      result.at20,
                      result.seconds,
                      result.contacts,
                      result.shootingAt,
                      result.firstPass,
                      result.at20 - result.firstPass,
                      balls.size()));
              if (auto.equals("James")) {
                jamesTotal += result.at20;
                jamesLayout += result.at20;
                if (layout > 0 && layout < 6) jamesSecond += result.at20 - result.firstPass;
              }
              if (auto.equals("Dynamic real UNet")) {
                unetTotal += result.at20;
                unetLayout += result.at20;
                if (layout > 0 && layout < 6) unetSecond += result.at20 - result.firstPass;
              }
              if (auto.equals("UNet blind")) blindTotal += result.at20;
            }
          }
        if (unetLayout > jamesLayout) wins++;
        System.out.printf(
            "layout=%d balls=%d at20 James=%d UNet=%d%n",
            layout, balls.size(), jamesLayout, unetLayout);
      }
      java.nio.file.Files.createDirectories(java.nio.file.Path.of("build/reports"));
      java.nio.file.Files.writeString(
          java.nio.file.Path.of("build/reports/unet-benchmark.csv"), csv);
      System.out.print(csv);
      assertTrue(
          unetSecond > jamesSecond,
          "Outward second-pass pickups: UNet=" + unetSecond + " James=" + jamesSecond);
      assertTrue(wins >= 3, "Must outperform James on at least three layouts");
      assertTrue(
          unetTotal > blindTotal * 1.10,
          "Targeting must improve pickups over the same route without detections");
      assertTrue(
          unetTotal > jamesTotal, "20-second pickups: UNet=" + unetTotal + " James=" + jamesTotal);
    } finally {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }

  private void runAuto(boolean red, boolean mirror, boolean detected) {
    runRoutine(red, mirror, detected, "Dynamic real UNet", null, -1);
  }

  private Result runRoutine(
      boolean red,
      boolean mirror,
      boolean detected,
      String label,
      java.util.List<Translation2d> balls,
      int layout) {
    var routine =
        Autos.AUTO_ROUTINES.stream()
            .filter(a -> a.displayLabel().equals(label))
            .findFirst()
            .orElseThrow();
    var startPath = new Path(routine.pathNames().get(0));
    if (mirror) startPath.mirror();
    if (red) startPath.flip();
    var c = TunerConstants.FrontLeft;
    var config =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(SimSwerveConstants.ROBOT_MASS)
            .withBumperSize(SimSwerveConstants.BUMPER_LENGTH_X, SimSwerveConstants.BUMPER_LENGTH_Y)
            .withCustomModuleTranslations(
                new Translation2d[] {
                  new Translation2d(c.LocationX, c.LocationY),
                  new Translation2d(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                  new Translation2d(
                      TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                  new Translation2d(
                      TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
                })
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    SimSwerveConstants.DRIVE_MOTOR_WHEEL,
                    SimSwerveConstants.STEER_MOTOR_WHEEL,
                    c.DriveMotorGearRatio,
                    SimSwerveConstants.STEER_MOTOR_GEAR_RATIO,
                    SimSwerveConstants.DRIVE_FRICTION_VOLTAGE,
                    SimSwerveConstants.STEER_FRICTION_VOLTAGE,
                    Meters.of(c.WheelRadius),
                    SimSwerveConstants.STEER_INERTIA,
                    SimSwerveConstants.WHEEL_COF));
    var body = new SwerveDriveSimulation(config, startPath.getStartPose());
    var drive = new SelfControlledSwerveDriveSimulation(body);
    var arena = new CollisionTrackingArena(body);
    SimulatedArena.overrideInstance(arena);
    SimulatedArena.overrideSimulationTimings(Seconds.of(0.002), 1);
    Subsystem requirement = new Subsystem() {};
    ChassisSpeeds[] demand = {new ChassisSpeeds()};
    var fuel = new FuelSim();
    fuel.configureOutwardPush(layout, mirror, red);
    boolean[] intaking = {false};
    int[] collected = {0}, at20 = {0};
    fuel.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        drive::getActualPoseInSimulationWorld,
        () ->
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getActualSpeedsRobotRelative(),
                drive.getActualPoseInSimulationWorld().getRotation()));
    fuel.registerIntake(
        -Dimensions.FULL_LENGTH,
        Dimensions.FULL_LENGTH / 2,
        -Dimensions.FULL_WIDTH / 6,
        Dimensions.FULL_WIDTH / 6,
        () -> intaking[0],
        () -> collected[0]++);
    if (balls != null)
      for (var ball : balls) {
        var point =
            mirror
                ? new Translation2d(
                    ball.getX(), com.pathplanner.lib.util.FlippingUtil.fieldSizeY - ball.getY())
                : ball;
        if (red) point = com.pathplanner.lib.util.FlippingUtil.flipFieldPosition(point);
        fuel.spawnFuel(new Translation3d(point.getX(), point.getY(), 0.075), new Translation3d());
      }
    FollowPath.registerEventTrigger("intakeDown", () -> intaking[0] = true);
    FollowPath.registerEventTrigger("shoot", () -> intaking[0] = false);
    FollowPath.registerEventTrigger("neutral", () -> intaking[0] = false);
    try (var nt = NetworkTableInstance.create();
        var cos =
            new CosBallPathingSim(
                drive::getActualPoseInSimulationWorld,
                nt,
                balls == null ? null : fuel::getGroundFuelPositions)) {
      nt.getTable("Sim/COS").getEntry("Detected").setBoolean(detected);
      cos.reset(mirror);
      double startedAt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      double shootingAt = Double.NaN;
      int firstPass = 0;
      for (var name : routine.pathNames()) {
        if (name.equals("H_UNet"))
          shootingAt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startedAt;
        var builder =
            new FollowPath.Builder(
                requirement,
                drive::getActualPoseInSimulationWorld,
                drive::getActualSpeedsRobotRelative,
                s -> demand[0] = Autos.limitSpeeds(s),
                new PIDController(5, 0, 0),
                new PIDController(3, 0, 0),
                new PIDController(0, 0, 0.1));
        Command command;
        if (name.equals("dynamic:hub")) {
          command =
              new DynamicSegmentHub(drive::getActualPoseInSimulationWorld, mirror, true)
                  .getCommand(builder);
        } else if (name.equals("dynamic:unet")) {
          command =
              new UnetCollectSegment(
                      requirement,
                      request -> {
                        if (request
                            instanceof com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric r)
                          demand[0] = new ChassisSpeeds(r.VelocityX, r.VelocityY, r.RotationalRate);
                        else demand[0] = new ChassisSpeeds();
                      },
                      nt,
                      () -> Commands.runOnce(() -> intaking[0] = true),
                      drive::getActualPoseInSimulationWorld,
                      () ->
                          red
                              ? edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                              : edu.wpi.first.wpilibj.DriverStation.Alliance.Blue,
                      mirror)
                  .getCommand(builder);
        } else {
          command =
              builder
                  .withShouldFlip(() -> red)
                  .withShouldMirror(() -> mirror)
                  .build(new Path(name));
        }
        command.initialize();
        int steps = 0;
        while (!command.isFinished() && steps++ < 1500) {
          cos.periodic();
          command.execute();
          drive.runChassisSpeeds(demand[0], new Translation2d(), false, false);
          for (int tick = 0; tick < 10; tick++) {
            arena.simulationPeriodic();
            SimHooks.stepTiming(0.002);
            assertEquals(
                0,
                arena.getFieldContacts(),
                "Field collision during "
                    + name
                    + " red="
                    + red
                    + " mirror="
                    + mirror
                    + " pose="
                    + drive.getActualPoseInSimulationWorld());
          }
          drive.periodic();
          if (balls != null) fuel.stepSim();
          if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startedAt <= 20)
            at20[0] = collected[0];
        }
        assertTrue(
            command.isFinished(),
            "Stuck during " + name + " at " + drive.getActualPoseInSimulationWorld());
        command.end(false);
        if (name.equals("S_Normal")) firstPass = collected[0];
        if (name.equals("F_UNet_Turn") && balls == null) {
          var pose = drive.getActualPoseInSimulationWorld();
          assertTrue(
              (mirror ^ red ? -1 : 1) * pose.getRotation().getSin() > Math.cos(Math.toRadians(6)),
              "Robot must face alliance-relative positive Y (negative Y when mirrored) before collection");
        }
      }
      if (balls == null)
        System.out.printf(
            "red=%s mirror=%s detected=%s contacts=%d completed in %.2fs%n",
            red,
            mirror,
            detected,
            arena.getFieldContacts(),
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startedAt);
      return new Result(
          collected[0],
          at20[0],
          edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startedAt,
          arena.getFieldContacts(),
          shootingAt,
          firstPass);
    } finally {
      arena.shutDown();
    }
  }
}
