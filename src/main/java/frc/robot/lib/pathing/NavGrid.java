package frc.robot.lib.pathing;

import java.util.List;

// struct NavGrid {
//   std::vector<std::vector<Node>> grid;
//   double nodeSizeMeters;
// };
public class NavGrid {
  public List<List<Node>> grid;
  public double nodeSizeMeters;

  public NavGrid() {}

  public NavGrid(List<List<Node>> grid, double nodeSizeMeters) {
    this.grid = grid;
    this.nodeSizeMeters = nodeSizeMeters;
  }
}
