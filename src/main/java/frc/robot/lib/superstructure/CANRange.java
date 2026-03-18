package frc.robot.lib.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class CANRange {
  private final CANrange canRange;

  @Getter private final String name;

  private final StatusSignal<Distance> canrangePosition;
  private final StatusSignal<Double> canrangeSignalStrength;

  @Getter
  @AutoLogOutput(key = "{name}/Position")
  private Distance position;

  @Getter
  @AutoLogOutput(key = "{name}/Signal Strength")
  private double signalStrength;

  public CANRange(CANRangeConfig config) {
    this.name = config.NAME();
    canRange = new CANrange(config.ID(), config.BUS());

    canrangePosition = canRange.getDistance();
    canrangeSignalStrength = canRange.getSignalStrength();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, canrangePosition, canrangeSignalStrength);

    canRange.optimizeBusUtilization();
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(canrangePosition, canrangeSignalStrength);
    position = canrangePosition.getValue();
    signalStrength = canrangeSignalStrength.getValue();
  }
}
