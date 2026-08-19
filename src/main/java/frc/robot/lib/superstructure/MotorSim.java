package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Simulates a motor subsystem by following a trapezoid profile to the goal. */
public class MotorSim extends MotorIO {

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State state;
  private TrapezoidProfile.State goal;

  public MotorSim(MotorConfig config) {
    super(config);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.TALONFX_CONFIG().MotionMagic.MotionMagicCruiseVelocity,
                config.TALONFX_CONFIG().MotionMagic.MotionMagicAcceleration));
    state = new TrapezoidProfile.State(0, 0);
    goal = new TrapezoidProfile.State(0, 0);
  }

  @Override
  public void setVoltage(Voltage goalVoltage) {
    appliedVoltage = goalVoltage;
  }

  @Override
  public void setVelocity(AngularVelocity goalVelocity) {
    goal = new TrapezoidProfile.State(state.position, goalVelocity.in(RotationsPerSecond));
  }

  @Override
  public void setPosition(Angle goalPosition) {
    goal = new TrapezoidProfile.State(goalPosition.in(Rotations), 0);
  }

  @Override
  public void setPosition(Distance goalPosition) {
    goal = new TrapezoidProfile.State(goalPosition.in(Meters), 0);
  }

  @Override
  public void setPositionVoltage(Angle goalPosition) {
    setPosition(goalPosition);
  }

  @Override
  public void setPositionVoltage(Distance goalPosition) {
    setPosition(goalPosition);
  }

  @Override
  public void periodic() {
    state = profile.calculate(Constants.UPDATE_PERIOD.in(Seconds), state, goal);
    position = Rotations.of(state.position);
    velocity = RotationsPerSecond.of(state.velocity);
    supplyCurrent = Amps.of(10.0);
    statorCurrent = Amps.of(10.0);
    temperature = Celsius.of(20.0);
    connected = true;

    super.periodic();
  }

  @Override
  public void resetPosition(Angle newPosition) {
    position = newPosition;
    state.position = position.in(Rotations);
  }

  @Override
  public void resetPosition(Distance newPosition) {
    position = Rotations.of(newPosition.in(Meters));
    state.position = position.in(Rotations);
  }

  @Override
  public void setSlot(int slot) {
    // no-op
  }

  @Override
  public void setCoast() {
    setVelocity(RotationsPerSecond.zero());
  }
}
