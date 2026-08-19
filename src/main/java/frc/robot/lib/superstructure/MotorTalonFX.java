package frc.robot.lib.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class MotorTalonFX extends MotorIO {
  protected Optional<CANcoder> cancoder = Optional.empty();
  protected final TalonFX motor;

  protected final VoltageOut voltageRequest;
  protected final VelocityVoltage velocityRequest;
  protected final DynamicMotionMagicVoltage dynamicMotionMagicPositionRequest;
  protected final PositionVoltage positionVoltageRequest;

  protected StatusSignal<Angle> positionSignal;
  protected StatusSignal<AngularVelocity> velocitySignal;
  protected StatusSignal<Voltage> appliedVoltageSignal;
  protected StatusSignal<Current> supplyCurrentSignal;
  protected StatusSignal<Current> statorCurrentSignal;
  protected StatusSignal<Temperature> temperatureSignal;
  protected Optional<StatusSignal<Angle>> absolutePositionSignal;
  protected int currentSlot = 0;

  public MotorTalonFX(MotorConfig motorConfig, Optional<CANcoderConfig> optionalCancoderConfig) {
    super(motorConfig);

    this.cancoderConfig = optionalCancoderConfig;

    MotionMagicConfigs mmConfigs = motorConfig.TALONFX_CONFIG().MotionMagic;

    voltageRequest = new VoltageOut(0.0).withEnableFOC(motorConfig.FOC());
    velocityRequest = new VelocityVoltage(0.0).withEnableFOC(motorConfig.FOC());
    positionVoltageRequest = new PositionVoltage(0.0).withEnableFOC(motorConfig.FOC());

    dynamicMotionMagicPositionRequest =
        new DynamicMotionMagicVoltage(
            0, mmConfigs.MotionMagicCruiseVelocity, mmConfigs.MotionMagicAcceleration);

    motor = new TalonFX(motorConfig.ID(), motorConfig.BUS());

    TalonFXConfiguration talonfxConfig = motorConfig.TALONFX_CONFIG();
    optionalCancoderConfig.ifPresent(
        cancoderConfig -> {
          talonfxConfig.Feedback.FeedbackRemoteSensorID = cancoderConfig.ID();
          talonfxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
          talonfxConfig.Feedback.RotorToSensorRatio = cancoderConfig.MOTOR_TO_SENSOR_RATIO();

          cancoder = Optional.of(new CANcoder(cancoderConfig.ID(), cancoderConfig.BUS()));
          cancoder.get().getConfigurator().apply(cancoderConfig.CANCODER_CONFIG());

          absolutePositionSignal = Optional.of(cancoder.get().getAbsolutePosition());
          BaseStatusSignal.setUpdateFrequencyForAll(100.0, absolutePositionSignal.get());
        });
    motor.getConfigurator().apply(talonfxConfig);

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    appliedVoltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        positionSignal,
        velocitySignal,
        appliedVoltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temperatureSignal);

    motor.optimizeBusUtilization();
  }

  public MotorTalonFX(MotorConfig motorConfig) {
    this(motorConfig, Optional.empty());
  }

  @Override
  public void setVoltage(Voltage goalVoltage) {
    motor.setControl(voltageRequest.withOutput(goalVoltage.in(Volts)));
  }

  @Override
  public void setVelocity(AngularVelocity goalVelocity) {
    motor.setControl(
        velocityRequest.withVelocity(goalVelocity.in(RotationsPerSecond)).withSlot(currentSlot));
  }

  @Override
  public void setPosition(Angle goalPosition) {
    setDynamicMotionMagicPosition(goalPosition.in(Rotations));
  }

  @Override
  public void setPosition(Distance goalPosition) {
    setDynamicMotionMagicPosition(goalPosition.in(Meters));
  }

  @Override
  public void setPositionVoltage(Angle goalPosition) {
    setPositionVoltage(goalPosition.in(Rotations));
  }

  @Override
  public void setPositionVoltage(Distance goalPosition) {
    setPositionVoltage(goalPosition.in(Meters));
  }

  private void setDynamicMotionMagicPosition(double goalPosition) {
    motor.setControl(
        dynamicMotionMagicPositionRequest
            .withPosition(goalPosition)
            .withFeedForward(feedforward)
            .withEnableFOC(motorConfig.FOC())
            .withSlot(currentSlot));
  }

  private void setPositionVoltage(double goalPosition) {
    motor.setControl(
        positionVoltageRequest
            .withPosition(goalPosition)
            .withFeedForward(feedforward)
            .withEnableFOC(motorConfig.FOC())
            .withSlot(currentSlot));
  }

  @Override
  public void setCoast() {
    motor.setControl(new CoastOut());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        appliedVoltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        temperatureSignal);

    position = positionSignal.getValue();
    velocity = velocitySignal.getValue();
    appliedVoltage = appliedVoltageSignal.getValue();
    supplyCurrent = supplyCurrentSignal.getValue();
    statorCurrent = statorCurrentSignal.getValue();
    temperature = temperatureSignal.getValue();
    connected = BaseStatusSignal.isAllGood(positionSignal, velocitySignal, appliedVoltageSignal);

    if (cancoder.isPresent()) {
      BaseStatusSignal.refreshAll(absolutePositionSignal.get());
      absolutePosition = absolutePositionSignal.get().getValue();
    }

    super.periodic();
  }

  @Override
  public void resetPosition(Angle newPosition) {
    motor.setPosition(newPosition.in(Rotations));
  }

  @Override
  public void resetPosition(Distance newPosition) {
    motor.setPosition(newPosition.in(Meters));
  }

  @Override
  public void setSlot(int slot) {
    currentSlot = slot;
  }
}
