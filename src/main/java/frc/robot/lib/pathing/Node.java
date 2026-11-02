package frc.robot.lib.pathing;

// clang-format off
// struct Node {
//   uint x, y;
//   double cost = INFINITY;
//   bool visited = false;
//   bool obstacle = false;
//   char readable;
//   bool path = false;
//   Node* parent = nullptr;
//
//   auto operator==(const Node& other) const -> bool {
//     return x == other.x && y == other.y;
//   }
// };
// clang-format on
public class Node {
  public int x, y;
  public double cost = Double.POSITIVE_INFINITY;
  public boolean visited = false;
  public boolean obstacle = false;
  public char readable;
  public boolean path = false;
  public Node parent = null;

  @Override
  public boolean equals(Object other) {
    if (!(other instanceof Node)) {
      return false;
    }
    Node o = (Node) other;
    return x == o.x && y == o.y;
  }

  @Override
  public int hashCode() {
    return java.util.Objects.hash(x, y);
  }
}
