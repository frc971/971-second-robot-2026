package frc.robot.lib.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class Splines {

  // auto KnotVector(int n, int p) -> std::vector<double> { int length = n + p +
  // 1;
  //
  // std::vector<double> knots(length);
  //
  // for (int i = 1; i < length; ++i) { if (i < p + 1) { knots[i] = 0.0; } else
  // if (i >= n) { knots[i] = 1.0; } else { knots[i] = static_cast<double>(i -
  // p) / (n - p);
  // }
  // }
  //
  // return knots;
  // }
  public static List<Double> KnotVector(int n, int p) {
    int length = n + p + 1;

    List<Double> knots = new ArrayList<>(java.util.Collections.nCopies(length, 0.0));

    for (int i = 1; i < length; ++i) {
      if (i < p + 1) {
        knots.set(i, 0.0);
      } else if (i >= n) {
        knots.set(i, 1.0);
      } else {
        knots.set(i, (double) (i - p) / (n - p));
      }
    }

    return knots;
  }

  // auto Basis(int i, int p, double t, const std::vector<double>& knots) ->
  // double { if (p == 0) { if (knots[i] <= t && t < knots[i + 1]) { return 1.0;
  // } else { return 0.0;
  // }
  // }
  //
  // double left = 0.0; double denoml = knots[i + p] - knots[i]; if (denoml !=
  // 0) { left = (t - knots[i]) / denoml * Basis(i, p - 1, t, knots);
  // }
  //
  // double right = 0.0; double denomr = knots[i + p + 1] - knots[i + 1]; if
  // (denomr != 0) { right = (knots[i + p + 1] - t) / denomr * Basis(i + 1, p -
  // 1, t, knots);
  // }
  //
  // return left + right;
  // }
  public static double Basis(int i, int p, double t, List<Double> knots) {
    if (p == 0) {
      if (knots.get(i) <= t && t < knots.get(i + 1)) {
        return 1.0;
      } else {
        return 0.0;
      }
    }

    double left = 0.0;
    double denoml = knots.get(i + p) - knots.get(i);
    if (denoml != 0) {
      left = (t - knots.get(i)) / denoml * Basis(i, p - 1, t, knots);
    }

    double right = 0.0;
    double denomr = knots.get(i + p + 1) - knots.get(i + 1);
    if (denomr != 0) {
      right = (knots.get(i + p + 1) - t) / denomr * Basis(i + 1, p - 1, t, knots);
    }

    return left + right;
  }

  // auto EvaluatePosition(double t, const std::vector<std::pair<double,
  // double>>& controls, const std::vector<double>& knots, int p) ->
  // std::pair<double, double> { if (t >= knots.back()) { return
  // controls.back();
  // }
  //
  // int n = static_cast<int>(controls.size()); double x = 0.0, y = 0.0; for
  // (int i = 0; i < n; ++i) { double b = Basis(i, p, t, knots); x += b *
  // controls[i].first; y += b * controls[i].second;
  // }
  // return {x, y};
  // }
  public static Pair<Double, Double> EvaluatePosition(
      double t, List<Pair<Double, Double>> controls, List<Double> knots, int p) {
    if (t >= knots.get(knots.size() - 1)) {
      return controls.get(controls.size() - 1);
    }

    int n = (int) controls.size();
    double x = 0.0, y = 0.0;
    for (int i = 0; i < n; ++i) {
      double b = Basis(i, p, t, knots);
      x += b * controls.get(i).first;
      y += b * controls.get(i).second;
    }
    return new Pair<>(x, y);
  }

  // auto CreateSpline(const std::vector<std::vector<pathing::Node>>& grid,
  // Point start_point, Point target_point, double nodeSizeMeters, int samples)
  // -> SplineResult {
  //
  // std::vector<std::vector<pathing::Node>> gridCopy = grid;
  // std::vector<pathing::Node> path = BFS(gridCopy, start_point, target_point);
  //
  // if (path.empty()) { LOG(INFO) << "BFS returned no path"; return {};
  // }
  //
  // std::vector<std::pair<double, double>> control_points; std::vector<double>
  // knots; std::vector<frc::Pose2d> spline_points; std::vector<double>
  // spline_params; uint p;
  //
  // control_points.reserve(path.size()); for (const pathing::Node& node : path)
  // { control_points.emplace_back(node.x * nodeSizeMeters, node.y *
  // nodeSizeMeters);
  // }
  //
  // uint numControls = control_points.size(); if (numControls < 4) { return {};
  // }
  //
  // p = 3; if (numControls <= p) { p = numControls - 1;
  // }
  //
  // knots = KnotVector(numControls, p);
  //
  // for (int t = 0; t <= samples; t += 1) { double t_real = t /
  // static_cast<double>(samples); auto [x, y] = EvaluatePosition(t_real,
  // control_points, knots, p); spline_points.emplace_back(units::meter_t{x},
  // units::meter_t{y}, 0_rad); spline_params.emplace_back(t_real);
  // }
  // auto first_deriv_controls = FiniteDifferences(control_points, knots, p, 1);
  // return {spline_points, control_points, first_deriv_controls, knots,
  // spline_params, p};
  // }
  //
  // NOTE: LOG(INFO) is a custom macro from src/utils/log.h in the source repo
  // and has no equivalent here; replaced with System.out.println as the closest
  // literal placeholder.
  public static SplineResult CreateSpline(
      List<List<Node>> grid,
      Point start_point,
      Point target_point,
      double nodeSizeMeters,
      int samples) {

    List<List<Node>> gridCopy = grid;
    List<Node> path = Pathfinding.BFS(gridCopy, start_point, target_point);

    if (path.isEmpty()) {
      System.out.println("BFS returned no path");
      return new SplineResult();
    }

    List<Pair<Double, Double>> control_points = new ArrayList<>();
    List<Double> knots;
    List<Pose2d> spline_points = new ArrayList<>();
    List<Double> spline_params = new ArrayList<>();
    int p;

    for (Node node : path) {
      control_points.add(new Pair<>(node.x * nodeSizeMeters, node.y * nodeSizeMeters));
    }

    int numControls = control_points.size();
    if (numControls < 4) {
      return new SplineResult();
    }

    p = 3;
    if (numControls <= p) {
      p = numControls - 1;
    }

    knots = KnotVector(numControls, p);

    for (int t = 0; t <= samples; t += 1) {
      double t_real = t / (double) samples;
      Pair<Double, Double> xy = EvaluatePosition(t_real, control_points, knots, p);
      double x = xy.first;
      double y = xy.second;
      spline_points.add(new Pose2d(x, y, Rotation2d.kZero));
      spline_params.add(t_real);
    }
    return new SplineResult(spline_points, control_points, knots, spline_params, p);
  }
}
