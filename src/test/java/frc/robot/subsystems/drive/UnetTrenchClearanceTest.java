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

}
