package frc.robot.lib.superstructure;

public class AngularSubsystem extends MotorSubsystem {
  public AngularSubsystem(MotorIO io) {
    super(io);
  }

  public AngularSubsystem(MotorConfig motorConfig) {
    super(motorConfig);
  }

  public AngularSubsystem(MotorConfig motorConfig, CANcoderConfig cancoderConfig) {
    super(motorConfig, cancoderConfig);
  }
}
