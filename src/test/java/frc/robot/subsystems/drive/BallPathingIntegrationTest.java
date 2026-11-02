package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

class BallPathingIntegrationTest {
  private SwerveRequest tick(Drive drive, CommandSwerveDrivetrain drivetrain) {
    DriverStationSim.notifyNewData();
    clearInvocations(drivetrain);
    drive.periodic();
    var request = ArgumentCaptor.forClass(SwerveRequest.class);
    verify(drivetrain).setRequest(request.capture());
    return request.getValue();
  }

  @Test
  void driverAHoldReleaseTimeoutAndModeTransitions() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    var driver = new XboxControllerSim(Controllers.DRIVER.getHID());
    var drivetrain = mock(CommandSwerveDrivetrain.class);
    var state = new SwerveDriveState();
    state.Pose = new Pose2d();
    when(drivetrain.getState()).thenReturn(state);
    var drive = new Drive(drivetrain);
    double[] goal = {1.2, -0.4, 2, 0.5, 3, 1, 0, 12, 0.2, -1.5};
    try (var publisher =
        NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/COS/BallVelocityGoal")
            .publish(PubSubOption.keepDuplicates(true))) {
      DriverStationSim.setEnabled(true);
      driver.setLeftY(-0.5);
      assertInstanceOf(SwerveRequest.FieldCentric.class, tick(drive, drivetrain));
      driver.setAButton(true);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, tick(drive, drivetrain));
      publisher.set(goal);
      var request = assertInstanceOf(SwerveRequest.RobotCentric.class, tick(drive, drivetrain));
      assertEquals(1.2, request.VelocityX);
      assertEquals(-0.4, request.VelocityY);
      assertEquals(-1.5, request.RotationalRate);
      // Holding A must continue applying pathing every cycle, without another button edge.
      assertInstanceOf(SwerveRequest.RobotCentric.class, tick(drive, drivetrain));
      driver.setAButton(false);
      var manual = assertInstanceOf(SwerveRequest.FieldCentric.class, tick(drive, drivetrain));
      assertTrue(manual.VelocityX > 0);
      driver.setAButton(true);
      publisher.set(goal);
      Timer.delay(0.3);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, tick(drive, drivetrain));
      publisher.set(goal); // Identical stationary commands must refresh freshness.
      assertInstanceOf(SwerveRequest.RobotCentric.class, tick(drive, drivetrain));
      goal[5] = 0;
      publisher.set(goal);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, tick(drive, drivetrain));
      goal[5] = 1;
      publisher.set(goal);
      DriverStationSim.setEnabled(false);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, tick(drive, drivetrain));
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      drive.setDriveMode(Drive.Mode.NONE);
      DriverStationSim.notifyNewData();
      clearInvocations(drivetrain);
      drive.periodic();
      verify(drivetrain, never()).setRequest(any());
      // Data received outside manual mode must not acquire a new timestamp on teleop entry.
      publisher.set(goal);
      Timer.delay(0.3);
      DriverStationSim.setAutonomous(false);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, tick(drive, drivetrain));
      publisher.set(goal);
      assertInstanceOf(SwerveRequest.RobotCentric.class, tick(drive, drivetrain));
    } finally {
      driver.setAButton(false);
      driver.setLeftY(0);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void publishesUnchangedPoseSamples() {
    try (var instance = NetworkTableInstance.create();
        var subscriber =
            instance
                .getStructTopic("/Drive/Pose", Pose2d.struct)
                .subscribe(
                    new Pose2d(), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        var publisher = new DrivePosePublisher(instance)) {
      publisher.publish(new Pose2d());
      publisher.publish(new Pose2d());
      assertEquals(2, subscriber.readQueue().length);
    }
  }
}
