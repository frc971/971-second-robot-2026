package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

public class HubShiftUtil {

  public enum ShiftEnum {
    TRANSITION, // 0–10s:   both active
    SHIFT1, // 10–35s:  loser active
    SHIFT2, // 35–60s:  winner active
    SHIFT3, // 60–85s:  loser active
    SHIFT4, // 85–110s: winner active
    ENDGAME, // 110–140s: both active
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift,
      ShiftEnum nextShift,
      double elapsedTime,
      double remainingTime,
      boolean hubActive) {}

  // Shift boundaries (seconds elapsed since teleop start)
  private static final double[] SHIFT_START = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] SHIFT_END = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  // Whether the auto-LOSER's hub is active each shift
  private static final boolean[] LOSER_ACTIVE = {true, true, false, true, false, true};
  private static final boolean[] BOTH_ACTIVE = {true, false, false, false, false, true};

  public static final double AUTO_DURATION = 20.0;
  public static final double TELEOP_DURATION = 140.0;

  private static final double TIME_RESET_THRESHOLD = 3.0;

  private static Timer shiftTimer = new Timer();
  private static double shiftTimerOffset = 0.0;

  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  /** Call at the start of teleop. */
  public static void initialize() {
    shiftTimerOffset = 0.0;
    shiftTimer.restart();
  }

  /**
   * Returns the alliance whose hub is active FIRST in teleop (i.e. the auto loser). FMS
   * game-specific message: 'R' = Red won auto → Blue hub active first. 'B' = Blue won auto → Red
   * hub active first.
   */
  public static Alliance getAutoLoserAlliance() {
    Alliance self = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Manual override: true = we won auto, false = we lost auto
    Optional<Boolean> override = allianceWinOverride.get();
    if (override.isPresent()) {
      boolean weWon = override.get();
      return weWon ? (self == Alliance.Blue ? Alliance.Red : Alliance.Blue) : self;
    }

    // FMS game-specific message
    String msg = DriverStation.getGameSpecificMessage();
    if (!msg.isEmpty()) {
      char c = msg.charAt(0);
      if (c == 'R') return Alliance.Blue; // Red won → Blue lost → Blue active first
      if (c == 'B') return Alliance.Red; // Blue won → Red lost → Red active first
    }

    // Default: assume we lost auto
    return self;
  }

  /** Returns whether our hub is active in the given shift index. */
  private static boolean isOurHubActive(int shiftIndex) {
    if (BOTH_ACTIVE[shiftIndex]) return true;

    Alliance self = DriverStation.getAlliance().orElse(Alliance.Blue);
    Alliance autoLoser = getAutoLoserAlliance();
    boolean loserIsUs = (autoLoser == self);
    // LOSER_ACTIVE[i] tells us if the loser's hub is active in shift i
    return loserIsUs ? LOSER_ACTIVE[shiftIndex] : !LOSER_ACTIVE[shiftIndex];
  }

  public static ShiftInfo getShiftInfo() {
    if (DriverStation.isAutonomousEnabled()) {
      double elapsed = shiftTimer.get();
      return new ShiftInfo(
          ShiftEnum.AUTO, ShiftEnum.TRANSITION, elapsed, AUTO_DURATION - elapsed, true);
    }

    if (!DriverStation.isEnabled()) {
      return new ShiftInfo(ShiftEnum.DISABLED, ShiftEnum.DISABLED, 0.0, 0.0, false);
    }

    // Sync timer to FMS match time if drift exceeds threshold
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double fieldTime = TELEOP_DURATION - DriverStation.getMatchTime();

    if (DriverStation.isFMSAttached()
        && fieldTime <= 135.0
        && Math.abs(fieldTime - currentTime) >= TIME_RESET_THRESHOLD) {
      shiftTimerOffset += currentTime - fieldTime;
      currentTime = fieldTime;
    }

    // Find current shift
    int idx = SHIFT_START.length - 1; // default to last (endgame)
    for (int i = 0; i < SHIFT_START.length; i++) {
      if (currentTime >= SHIFT_START[i] && currentTime < SHIFT_END[i]) {
        idx = i;
        break;
      }
    }

    double elapsed = currentTime - SHIFT_START[idx];
    double remaining = SHIFT_END[idx] - currentTime;
    ShiftEnum shift = ShiftEnum.values()[idx];
    ShiftEnum nextShift =
        (idx + 1 < SHIFT_START.length) ? ShiftEnum.values()[idx + 1] : ShiftEnum.DISABLED;
    boolean active = isOurHubActive(idx);

    return new ShiftInfo(shift, nextShift, elapsed, remaining, active);
  }
}
