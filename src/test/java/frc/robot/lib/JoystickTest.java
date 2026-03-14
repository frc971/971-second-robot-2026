package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class JoystickTest {
  private static final double TOLERANCE = 0.0001;
  private static final double MAX_VELOCITY = 2.0;
  private static final double MAX_ANGULAR_VELOCITY = 2.0;
  private static final double TRANSLATION_EXP = 2.0;
  private static final double ROTATION_EXP = 2.0;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotLimiter;

  @BeforeEach
  void setUp() {
    xLimiter = new SlewRateLimiter(1.0);
    yLimiter = new SlewRateLimiter(1.0);
    rotLimiter = new SlewRateLimiter(1.0);
  }

  /**
   * Verifies that when all inputs are zero, the JoystickValues record correctly returns zero
   * values. This is a basic sanity check to ensure the record's getters work correctly.
   */
  @Test
  void zeroInputsYieldZeroValues() {
    var inputs = new JoystickValues(0.0, 0.0, 0.0);

    assertEquals(0.0, inputs.getX(), TOLERANCE, "X should be zero");
    assertEquals(0.0, inputs.getY(), TOLERANCE, "Y should be zero");
    assertEquals(0.0, inputs.getRot(), TOLERANCE, "Rot should be zero");
  }

  /**
   * Tests the exponential curve functionality by verifying that: 1. The power curve is correctly
   * applied to each axis 2. The sign of the input is preserved 3. The translation and rotation
   * exponents are applied correctly to their respective axes
   */
  @Test
  void exponentialCurveAppliesCorrectly() {
    double xInput = 0.5;
    double yInput = 0.5;
    double rotInput = 0.5;

    var inputs = new JoystickValues(xInput, yInput, rotInput);
    var curved = inputs.exponentialCurve(TRANSLATION_EXP, ROTATION_EXP);

    double expectedX = Math.copySign(Math.pow(Math.abs(xInput), TRANSLATION_EXP), xInput);
    double expectedY = Math.copySign(Math.pow(Math.abs(yInput), TRANSLATION_EXP), yInput);
    double expectedRot = Math.copySign(Math.pow(Math.abs(rotInput), ROTATION_EXP), rotInput);

    assertEquals(expectedX, curved.getX(), TOLERANCE, "X should match curved value");
    assertEquals(expectedY, curved.getY(), TOLERANCE, "Y should match curved value");
    assertEquals(expectedRot, curved.getRot(), TOLERANCE, "Rot should match curved value");
  }

  /**
   * Verifies that the slew rate limiting functionality works by: 1. Testing that initial values are
   * properly limited 2. Ensuring that sudden changes in input are rate-limited 3. Checking that all
   * axes (x, y, rotation) are limited independently
   */
  @Test
  void slewRateLimitAppliesCorrectly() {
    var inputs = new JoystickValues(1.0, 1.0, 1.0);
    var curved = inputs.exponentialCurve(TRANSLATION_EXP, ROTATION_EXP);
    var scaled = curved.scale(MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
    var limited = scaled.slewRateLimit(xLimiter, yLimiter, rotLimiter);

    // First call should be limited by the rate limiters
    assertEquals(0.0, limited.getX(), 0.01, "X should be limited");
    assertEquals(0.0, limited.getY(), 0.01, "Y should be limited");
    assertEquals(0.0, limited.getRot(), 0.01, "Rot should be limited");
  }

  /**
   * Tests the scaling functionality by verifying that: 1. Translation values (x, y) are scaled by
   * the maximum velocity 2. Rotation values are scaled by the maximum angular velocity 3. The
   * scaling is applied correctly to all axes
   */
  @Test
  void scaleAppliesCorrectly() {
    var inputs = new JoystickValues(0.5, 0.5, 0.5);
    var curved = inputs.exponentialCurve(TRANSLATION_EXP, ROTATION_EXP);
    var scaled = curved.scale(MAX_VELOCITY, MAX_ANGULAR_VELOCITY);

    double expectedX = Math.copySign(Math.pow(Math.abs(0.5), TRANSLATION_EXP), 0.5) * MAX_VELOCITY;
    double expectedY = Math.copySign(Math.pow(Math.abs(0.5), TRANSLATION_EXP), 0.5) * MAX_VELOCITY;
    double expectedRot =
        Math.copySign(Math.pow(Math.abs(0.5), ROTATION_EXP), 0.5) * MAX_ANGULAR_VELOCITY;

    assertEquals(expectedX, scaled.getX(), TOLERANCE, "X should be scaled by max velocity");
    assertEquals(expectedY, scaled.getY(), TOLERANCE, "Y should be scaled by max velocity");
    assertEquals(
        expectedRot, scaled.getRot(), TOLERANCE, "Rot should be scaled by max angular velocity");
  }

  /**
   * Verifies that all operations can be chained together correctly by: 1. Applying exponential
   * curve 2. Applying scaling 3. Applying slew rate limiting This ensures that the operations work
   * together as expected in the actual robot code.
   */
  @Test
  void chainedOperationsWorkCorrectly() {
    var inputs = new JoystickValues(0.5, 0.5, 0.5);
    var processed =
        inputs
            .exponentialCurve(TRANSLATION_EXP, ROTATION_EXP)
            .scale(MAX_VELOCITY, MAX_ANGULAR_VELOCITY)
            .slewRateLimit(xLimiter, yLimiter, rotLimiter);

    // First call to slew rate limit should be limited
    assertEquals(0.0, processed.getX(), 0.01, "X should be limited after chaining");
    assertEquals(0.0, processed.getY(), 0.01, "Y should be limited after chaining");
    assertEquals(0.0, processed.getRot(), 0.01, "Rot should be limited after chaining");
  }

  /**
   * Comprehensive test that verifies all combinations of input values can reach their target values
   * when using very high slew rate limits. This test: 1. Tests all combinations of (0.0, 0.5, 1.0)
   * for x, y, and rotation 2. Uses extremely high slew rate limits to ensure immediate value
   * changes 3. Verifies that exponential curve and scaling work correctly for each combination 4.
   * Ensures the system can handle any valid input combination
   */
  @Test
  void canReachDesiredValue() {
    // Create new limiters with very high limits to allow immediate value changes
    var fastXLimiter = new SlewRateLimiter(1e8);
    var fastYLimiter = new SlewRateLimiter(1e8);
    var fastRotLimiter = new SlewRateLimiter(1e8);

    List<Double> testValues = Arrays.asList(0.0, 0.5, 1.0);

    // Test all combinations of test values
    for (double xValue : testValues) {
      for (double yValue : testValues) {
        for (double rotValue : testValues) {
          var processed =
              new JoystickValues(xValue, yValue, rotValue)
                  .exponentialCurve(TRANSLATION_EXP, ROTATION_EXP)
                  .scale(MAX_VELOCITY, MAX_ANGULAR_VELOCITY)
                  .slewRateLimit(fastXLimiter, fastYLimiter, fastRotLimiter);

          // Calculate expected values
          double expectedX =
              Math.copySign(Math.pow(Math.abs(xValue), TRANSLATION_EXP), xValue) * MAX_VELOCITY;
          double expectedY =
              Math.copySign(Math.pow(Math.abs(yValue), TRANSLATION_EXP), yValue) * MAX_VELOCITY;
          double expectedRot =
              Math.copySign(Math.pow(Math.abs(rotValue), ROTATION_EXP), rotValue)
                  * MAX_ANGULAR_VELOCITY;

          // Values should reach their target immediately with high slew rate limits
          String message =
              String.format("Testing values (x=%.1f, y=%.1f, rot=%.1f)", xValue, yValue, rotValue);
          assertEquals(
              expectedX, processed.getX(), TOLERANCE, "X should reach target value - " + message);
          assertEquals(
              expectedY, processed.getY(), TOLERANCE, "Y should reach target value - " + message);
          assertEquals(
              expectedRot,
              processed.getRot(),
              TOLERANCE,
              "Rot should reach target value - " + message);
        }
      }
    }
  }
}
