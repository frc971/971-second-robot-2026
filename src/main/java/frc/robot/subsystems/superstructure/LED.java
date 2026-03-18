package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;

public class LED {
  private static final int CANDLE_ID = 8;
  private static final int LEFT_LED_START = 0;
  private static final int LEFT_LED_END = 14;
  private static final int RIGHT_LED_START = 15;
  private static final int RIGHT_LED_END = 50;

  private static final Color LED_COLOR_NOT_LOCKED = Color.kOrange;
  private static final Color LED_COLOR_LOCKED_READY = Color.kGreen;
  private static final Color LED_COLOR_SHOOTING = Color.kBlue;
  private static final Color LED_COLOR_KILLED = Color.kYellow;

  private final CANdle candle;
  private final SolidColor leftSolidColorRequest;
  private final SolidColor rightSolidColorRequest;

  public LED() {
    candle = new CANdle(CANDLE_ID, new CANBus("rio"));
    candle
        .getConfigurator()
        .apply(
            new CANdleConfiguration()
                .withLED(
                    new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0)));

    leftSolidColorRequest = new SolidColor(LEFT_LED_START, LEFT_LED_END);
    rightSolidColorRequest = new SolidColor(RIGHT_LED_START, RIGHT_LED_END);
  }

  public void updateTurretSegments(
      boolean leftKilled,
      boolean rightKilled,
      boolean shootCommandActive,
      boolean leftLocked,
      boolean rightLocked) {
    boolean leftShooting = shootCommandActive && leftLocked;
    boolean rightShooting = shootCommandActive && rightLocked;

    setSegmentColor(leftSolidColorRequest, getSegmentColor(leftKilled, leftLocked, leftShooting));
    setSegmentColor(
        rightSolidColorRequest, getSegmentColor(rightKilled, rightLocked, rightShooting));
  }

  private Color getSegmentColor(boolean killed, boolean locked, boolean shooting) {
    if (killed) {
      return LED_COLOR_KILLED;
    }
    if (shooting) {
      return LED_COLOR_SHOOTING;
    }
    if (locked) {
      return LED_COLOR_LOCKED_READY;
    }
    return LED_COLOR_NOT_LOCKED;
  }

  private void setSegmentColor(SolidColor request, Color color) {
    candle.setControl(request.withColor(new RGBWColor(color)));
  }
}
