package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class MotorWithFollowerTalonFX extends MotorTalonFX {
  protected final MotorConfig[] followerConfigs;
  protected final TalonFX[] followerMotors;

  protected StatusSignal<Angle>[] followerPositionSignals;
  protected StatusSignal<AngularVelocity>[] followerVelocitySignals;
  protected StatusSignal<Voltage>[] followerAppliedVoltageSignals;
  protected StatusSignal<Current>[] followerSupplyCurrentSignals;
  protected StatusSignal<Current>[] followerStatorCurrentSignals;
  protected StatusSignal<Temperature>[] followerTemperatureSignals;

  public MotorWithFollowerTalonFX(MotorConfig config, MotorConfig[] followerConfigs) {
    super(config);

    this.followerConfigs = followerConfigs;

    followerMotors = new TalonFX[followerConfigs.length];
    followerPositionSignals = new StatusSignal[followerConfigs.length];
    followerVelocitySignals = new StatusSignal[followerConfigs.length];
    followerAppliedVoltageSignals = new StatusSignal[followerConfigs.length];
    followerSupplyCurrentSignals = new StatusSignal[followerConfigs.length];
    followerStatorCurrentSignals = new StatusSignal[followerConfigs.length];
    followerTemperatureSignals = new StatusSignal[followerConfigs.length];

    for (int i = 0; i < followerConfigs.length; i++) {
      followerMotors[i] = new TalonFX(followerConfigs[i].ID(), followerConfigs[i].BUS());
      followerMotors[i].getConfigurator().apply(followerConfigs[i].TALONFX_CONFIG());

      StatusSignal<Angle> positionSignal = followerMotors[i].getPosition();
      StatusSignal<AngularVelocity> velocitySignal = followerMotors[i].getVelocity();
      StatusSignal<Voltage> appliedVoltageSignal = followerMotors[i].getMotorVoltage();
      StatusSignal<Current> supplyCurrentSignal = followerMotors[i].getSupplyCurrent();
      StatusSignal<Current> statorCurrentSignal = followerMotors[i].getStatorCurrent();
      StatusSignal<Temperature> temperatureSignal = followerMotors[i].getDeviceTemp();

      followerPositionSignals[i] = positionSignal;
      followerVelocitySignals[i] = velocitySignal;
      followerAppliedVoltageSignals[i] = appliedVoltageSignal;
      followerSupplyCurrentSignals[i] = supplyCurrentSignal;
      followerStatorCurrentSignals[i] = statorCurrentSignal;
      followerTemperatureSignals[i] = temperatureSignal;

      BaseStatusSignal.setUpdateFrequencyForAll(
          100.0,
          positionSignal,
          velocitySignal,
          appliedVoltageSignal,
          supplyCurrentSignal,
          statorCurrentSignal);
      BaseStatusSignal.setUpdateFrequencyForAll(4.0, temperatureSignal);

      followerMotors[i].setControl(
          new Follower(motor.getDeviceID(), followerConfigs[i].FOLLOWER_ALIGNMENT()));
      followerMotors[i].optimizeBusUtilization();
    }
  }

  public void periodic() {
    super.periodic();

    for (int i = 0; i < followerConfigs.length; i++) {
      BaseStatusSignal.refreshAll(
          followerPositionSignals[i],
          followerVelocitySignals[i],
          followerAppliedVoltageSignals[i],
          followerSupplyCurrentSignals[i],
          followerStatorCurrentSignals[i],
          followerTemperatureSignals[i]);

      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Position",
          UnitUtil.toDouble(followerPositionSignals[i].getValue(), followerConfigs[i].LOG_UNIT()));
      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Velocity",
          UnitUtil.toDouble(
              followerVelocitySignals[i].getValue(), followerConfigs[i].LOG_UNIT().per(Seconds)));
      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Applied Voltage",
          followerAppliedVoltageSignals[i].getValue());
      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Supply Current",
          followerSupplyCurrentSignals[i].getValue());
      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Stator Current",
          followerStatorCurrentSignals[i].getValue());
      Logger.recordOutput(
          followerConfigs[i].NAME() + "/Temperature", followerTemperatureSignals[i].getValue());
    }
  }

  public void resetPosition(Angle newPosition) {
    super.resetPosition(newPosition);
    for (TalonFX motor : followerMotors) {
      motor.setPosition(newPosition.in(Rotations));
    }
  }

  public void resetPosition(Distance newPosition) {
    super.resetPosition(newPosition);
    for (TalonFX motor : followerMotors) {
      motor.setPosition(newPosition.in(Meters));
    }
  }
}
