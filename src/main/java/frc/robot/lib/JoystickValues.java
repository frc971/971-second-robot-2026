package frc.robot.lib;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class JoystickValues {
  private double x = 0.0;
  private double y = 0.0;
  private double rot = 0.0;

  public JoystickValues() {}

  public JoystickValues(double x, double y, double rot) {
    setValues(x, y, rot);
  }

  public JoystickValues setValues(double x, double y, double rot) {
    this.x = x;
    this.y = y;
    this.rot = rot;
    return this;
  }

  public JoystickValues exponentialCurve(double translation_exp, double rotation_exp) {
    x = applyPowerCurve(x, translation_exp);
    y = applyPowerCurve(y, translation_exp);
    rot = applyPowerCurve(rot, rotation_exp);
    return this;
  }

  public JoystickValues slewRateLimit(
      SlewRateLimiter xLimiter, SlewRateLimiter yLimiter, SlewRateLimiter rotLimiter) {
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);
    return this;
  }

  public JoystickValues scale(double maxVelocity, double maxAngularVelocity) {
    x *= maxVelocity;
    y *= maxVelocity;
    rot *= maxAngularVelocity;
    return this;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getRot() {
    return rot;
  }

  /**
   * Applies a power curve to the input while preserving its sign. This creates a non-linear
   * response where small inputs result in smaller outputs, making fine control easier at low speeds
   * while still allowing full range at high speeds.
   *
   * @param input The input value to transform (-1.0 to 1.0)
   * @param exponent The power to raise the input to (typically > 1.0)
   * @return The transformed value with the same sign as the input
   */
  private static double applyPowerCurve(double input, double exponent) {
    return Math.copySign(Math.pow(Math.abs(input), exponent), input);
  }
}
