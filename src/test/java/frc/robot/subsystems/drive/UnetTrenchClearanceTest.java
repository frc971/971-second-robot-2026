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

  private void runAuto(boolean red, boolean mirror, boolean detected) {
    var routine =
        Autos.AUTO_ROUTINES.stream()
            .filter(a -> a.displayLabel().equals("Dynamic real UNet"))
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
    try (var nt = NetworkTableInstance.create();
        var cos = new CosBallPathingSim(drive::getActualPoseInSimulationWorld, nt)) {
      nt.getTable("Sim/COS").getEntry("Detected").setBoolean(detected);
      cos.reset(mirror);
      double startedAt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      for (var name : routine.pathNames()) {
        var builder =
            new FollowPath.Builder(
                requirement,
                drive::getActualPoseInSimulationWorld,
                drive::getActualSpeedsRobotRelative,
                s -> demand[0] = s,
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
                      Commands::none,
                      drive::getActualPoseInSimulationWorld,
                      () ->
                          red
                              ? edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                              : edu.wpi.first.wpilibj.DriverStation.Alliance.Blue)
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
        }
        assertTrue(
            command.isFinished(),
            "Stuck during " + name + " at " + drive.getActualPoseInSimulationWorld());
        command.end(false);
        if (name.equals("F_UNet_Turn")) {
          var pose = drive.getActualPoseInSimulationWorld();
          assertTrue(
              (mirror ^ red ? -1 : 1) * pose.getRotation().getSin() > Math.cos(Math.toRadians(6)),
              "Robot must face alliance-relative positive Y (negative Y when mirrored) before collection");
        }
      }
      System.out.printf(
          "red=%s mirror=%s detected=%s contacts=%d completed in %.2fs%n",
          red,
          mirror,
          detected,
          arena.getFieldContacts(),
          edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startedAt);
    } finally {
      arena.shutDown();
    }
  }
}
