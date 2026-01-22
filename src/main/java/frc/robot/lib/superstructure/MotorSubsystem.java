package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MotorSubsystem {
  protected final MotorIO io;

  private final String name;

  @Getter
  @AutoLogOutput(key = "{name}/Goal Voltage")
  protected Voltage goalVoltage;

  @Getter protected AngularVelocity goalVelocity;

  @Getter protected Angle goalPosition;

  public MotorSubsystem(MotorIO io) {
    this.io = io;
    this.name = io.getMotorConfig().NAME();
  }

  public MotorSubsystem(MotorConfig motorConfig) {
    this(RobotBase.isSimulation() ? new MotorSim(motorConfig) : new MotorTalonFX(motorConfig));
  }

  public MotorSubsystem(MotorConfig motorConfig, CANcoderConfig cancoderConfig) {
    this(
        RobotBase.isSimulation()
            ? new MotorSim(motorConfig)
            : new MotorTalonFX(motorConfig, Optional.of(cancoderConfig)));
  }

  public void periodic() {
    io.periodic();

    if (goalPosition != null)
      Logger.recordOutput(
          name + "/Goal Position", UnitUtil.toDouble(goalPosition, io.getMotorConfig().LOG_UNIT()));
    if (goalVelocity != null)
      Logger.recordOutput(
          name + "/Goal Velocity",
          UnitUtil.toDouble(goalVelocity, io.getMotorConfig().LOG_UNIT().per(Seconds)));
  }

  public void setVoltage(Voltage goalVoltage) {
    this.goalVoltage = goalVoltage;
    io.setVoltage(goalVoltage);
  }

  public void setPosition(Angle goalPosition) {
    this.goalPosition = goalPosition;
    io.setPosition(goalPosition);
  }

  public void setPosition(Distance goalPosition) {
    this.goalPosition = Rotations.of(goalPosition.in(Meters));
    io.setPosition(goalPosition);
  }

  public void setVelocity(AngularVelocity goalVelocity) {
    this.goalVelocity = goalVelocity;
    io.setVelocity(goalVelocity);
  }

  public Angle getPosition() {
    return io.position;
  }

  public AngularVelocity getVelocity() {
    return io.velocity;
  }

  public Voltage getAppliedVoltage() {
    return io.appliedVoltage;
  }

  public Current getSupplyCurrent() {
    return io.supplyCurrent;
  }

  public Current getStatorCurrent() {
    return io.statorCurrent;
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public void resetPosition(Distance newPosition) {
    io.resetPosition(newPosition);
  }
}
