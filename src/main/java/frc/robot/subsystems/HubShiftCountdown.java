package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/** Publishes countdown signals for the Driver Station computer to use when playing the audio clip. */
public class HubShiftCountdown {
  // The clip contains a full ten-second countdown, so it starts ten seconds before each shift.
  private static final int COUNTDOWN_SECONDS = 10;

  // Teleop time remaining when each clip should start. The values correspond to total match elapsed
  // times 20, 45, 70, 95, and 120 seconds, each ten seconds before a shift boundary.
  private static final double[] COUNTDOWN_STARTS_TELEOP_REMAINING = {140.0, 115.0, 90.0, 65.0, 40.0};

  // This topic is the link between the robot code and the Driver Station computer's Python listener.
  // The robot publishes the countdown number; the computer listens and plays the WAV at 10.
  private final IntegerPublisher countdownPublisher =
      NetworkTableInstance.getDefault()
          .getIntegerTopic("/DriverStation/HubShiftCountdown")
          .publish();

  // Remember the last value so the same number is not published on every 20 ms robot loop.
  private int lastPublishedCountdown = 0;

  /** Checks the teleop timer and publishes 10 through 1 during each countdown window. */
  public void update() {
    int countdown = 0;

    // Only announce shifts during teleop. Auto and disabled periods always publish no countdown.
    if (DriverStation.isTeleopEnabled()) {
      double teleopTimeRemaining = DriverStation.getMatchTime();

      // Find which countdown window contains the current time, if any.
      for (double countdownStart : COUNTDOWN_STARTS_TELEOP_REMAINING) {
        double timeUntilShift = teleopTimeRemaining - (countdownStart - COUNTDOWN_SECONDS);
        if (timeUntilShift > 0.0 && timeUntilShift <= COUNTDOWN_SECONDS) {
          // Round up so the published value steps cleanly from 10 down to 1.
          countdown = (int) Math.ceil(timeUntilShift);
          break;
        }
      }
    }

    // Publish only changes. A zero marks the end of a countdown window.
    if (countdown != lastPublishedCountdown) {
      countdownPublisher.set(countdown);
      lastPublishedCountdown = countdown;
    }
  }

  /** Releases the NetworkTables publisher during shutdown. */
  public void close() {
    countdownPublisher.close();
  }
}
