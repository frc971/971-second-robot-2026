package frc.robot.lib.simulation;
import static org.junit.jupiter.api.Assertions.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
class CosIntegrationTest {
  @Test void frozenBallsStayFixedAndCountOnce() throws Exception {
    var fuel = new FuelSim();
    fuel.registerRobot(1, 1, .2, () -> new edu.wpi.first.math.geometry.Pose2d(),
        () -> new edu.wpi.first.math.kinematics.ChassisSpeeds());
    fuel.registerIntake(-1, 1, -1, 1);
    CosIntegration.spawn(fuel, new ObjectMapper().readTree("{\"mode\":\"frozen-unet\",\"balls\":[[0.5,0,0.075],[2,0,0.2]]}"));
    fuel.stepSim(); fuel.stepSim();
    assertEquals(1, fuel.getCollected());
    assertArrayEquals(new double[]{0.5,0,0.075}, fuel.getFuelPositions().get(0));
    assertEquals(2, fuel.getFuelPositions().size());
  }
  @Test void exactPositionsAndEmptyLayout() throws Exception {
    var fuel = new FuelSim();
    CosIntegration.spawn(fuel, new ObjectMapper().readTree("{\"balls\":[[6.125,2.25,0.075],[7.5,3.0,0.2]]}"));
    var points = fuel.getFuelPositions();
    assertArrayEquals(new double[]{6.125,2.25,0.075}, points.get(0));
    assertArrayEquals(new double[]{7.5,3.0,0.2}, points.get(1));
    fuel.clearFuel();
    CosIntegration.spawn(fuel, new ObjectMapper().readTree("{\"balls\":[]}"));
    assertTrue(fuel.getFuelPositions().isEmpty());
  }
}
