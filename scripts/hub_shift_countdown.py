#!/usr/bin/env python3
"""Play the prerecorded hub-shift countdown when signaled by the robot."""

# Standard Python tools for command-line options, locating the audio file, and polling the topic.
import argparse
from pathlib import Path
import time

# Windows' built-in audio player; the Driver Station computer is expected to run Windows.
import winsound

# FRC's NetworkTables library, used to receive the countdown signal from the robot.
import ntcore


# Default FRC team number and the NetworkTables topic published by HubShiftCountdown.java.
TEAM_NUMBER = 971
TOPIC_NAME = "/DriverStation/HubShiftCountdown"

# Keep the WAV next to this script so it can be found no matter what folder the script is launched from.
COUNTDOWN_AUDIO = Path(__file__).with_name("hub_shift_countdown.wav")


def main() -> None:
    # Allow another team number to be supplied when launching the listener.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--team", type=int, default=TEAM_NUMBER, help="FRC team number")
    args = parser.parse_args()

    # Start a NetworkTables client and subscribe to the robot's countdown topic.
    nt_instance = ntcore.NetworkTableInstance.getDefault()
    nt_instance.startClient4("HubShiftCountdownSpeaker")
    nt_instance.setServerTeam(args.team)
    subscriber = nt_instance.getIntegerTopic(TOPIC_NAME).subscribe(0)

    # Keep track of the last number so the clip starts once when a new countdown begins.
    last_countdown = 0
    print(f"Listening for hub-shift countdown on team {args.team}. Press Ctrl+C to stop.")

    try:
        # Check for new values frequently. The robot publishes 10 at the start of each countdown.
        while True:
            countdown = int(subscriber.get())
            if countdown == 10 and last_countdown != 10:
                print("Playing the 10-second hub-shift countdown.")
                # Play asynchronously so this listener can continue checking NetworkTables.
                winsound.PlaySound(
                    str(COUNTDOWN_AUDIO), winsound.SND_FILENAME | winsound.SND_ASYNC
                )
            last_countdown = countdown
            time.sleep(0.02)
    except KeyboardInterrupt:
        # Ctrl+C is the normal way to stop the listener between matches.
        print("Stopping hub-shift countdown speaker.")
    finally:
        # Release the NetworkTables connection cleanly before exiting.
        subscriber.close()
        nt_instance.stopClient()


# Run main only when this file is launched directly, not when imported by another Python file.
if __name__ == "__main__":
    main()
