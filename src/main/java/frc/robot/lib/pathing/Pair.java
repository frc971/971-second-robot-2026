package frc.robot.lib.pathing;

// Java has no built-in equivalent of std::pair<A, B>, which is used
// throughout the original C++ (controls, velocity profile entries, etc.).
// This is a mechanical stand-in with the same .first/.second member access.
public class Pair<A, B> {
  public A first;
  public B second;

  public Pair() {}

  public Pair(A first, B second) {
    this.first = first;
    this.second = second;
  }
}
