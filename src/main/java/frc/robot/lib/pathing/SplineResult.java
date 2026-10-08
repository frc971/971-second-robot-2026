package frc.robot.lib.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

// struct SplineResult {
//   std::vector<frc::Pose2d> points;
//   std::vector<std::pair<double, double>> controls;
//   std::vector<double> knots;
//   std::vector<double> params;
//   uint p;
// };
public class SplineResult {
  public List<Pose2d> points = new ArrayList<>();
  public List<Pair<Double, Double>> controls = new ArrayList<>();
  public List<Double> knots = new ArrayList<>();
  public List<Double> params = new ArrayList<>();
  public int p;

  public SplineResult() {}

  public SplineResult(
      List<Pose2d> points,
      List<Pair<Double, Double>> controls,
      List<Double> knots,
      List<Double> params,
      int p) {
    this.points = points;
    this.controls = controls;
    this.knots = knots;
    this.params = params;
    this.p = p;
  }
}
