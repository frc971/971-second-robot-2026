package frc.robot.subsystems.drive;

import java.util.Optional;

record BallPathingGoal(
    double velocityX,
    double velocityY,
    double omegaRadiansPerSecond,
    double targetX,
    double targetY) {
  static Optional<BallPathingGoal> fromNetworkTables(
      double[] values, double maximumSpeed, double maximumAngularRate) {
    if (values.length < 10
        || values[5] == 0.0
        || !Double.isFinite(values[0])
        || !Double.isFinite(values[1])
        || !Double.isFinite(values[9])) {
      return Optional.empty();
    }

    double velocityX = values[0];
    double velocityY = values[1];
    double speed = Math.hypot(velocityX, velocityY);
    if (speed > maximumSpeed) {
      velocityX *= maximumSpeed / speed;
      velocityY *= maximumSpeed / speed;
    }
    double omega = Math.max(-maximumAngularRate, Math.min(maximumAngularRate, values[9]));
    return Optional.of(new BallPathingGoal(velocityX, velocityY, omega, values[2], values[3]));
  }
}
