package frc.robot.lib.pathing;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class Pathfinding {

  // auto GetGrid(const std::string& navgrid_path) -> NavGrid {
  //   std::ifstream file(navgrid_path);
  //   if (!file.is_open()) {
  //     LOG(FATAL) << "Failed to open navgrid: " << navgrid_path;
  //     return {};
  //   }
  //
  //   nlohmann::json data = nlohmann::json::parse(file);
  //   file.close();
  //
  //   const int GRID_H = data["grid"].size();
  //   const int GRID_W = data["grid"][0].size();
  //   double nodeSizeMeters = data["nodeSizeMeters"];
  //
  //   std::vector<std::vector<pathing::Node>> grid(
  //       GRID_H, std::vector<pathing::Node>(GRID_W));
  //   for (int y = 0; y < GRID_H; ++y) {
  //     for (int x = 0; x < GRID_W; ++x) {
  //       grid[y][x].x = x;
  //       grid[y][x].y = y;
  //       grid[y][x].obstacle = data["grid"][y][x];
  //     }
  //   }
  //   return {.grid = std::move(grid), .nodeSizeMeters = nodeSizeMeters};
  // }
  //
  // NOTE: LOG(FATAL) is a custom macro from src/utils/log.h in the source
  // repo and has no equivalent here; replaced with System.err.println as the
  // closest literal placeholder.
  public static NavGrid GetGrid(String navgrid_path) {
    File file = new File(navgrid_path);
    JsonNode data;
    try {
      data = new ObjectMapper().readTree(file);
    } catch (IOException e) {
      System.err.println("Failed to open navgrid: " + navgrid_path);
      return new NavGrid();
    }

    final int GRID_H = data.get("grid").size();
    final int GRID_W = data.get("grid").get(0).size();
    double nodeSizeMeters = data.get("nodeSizeMeters").asDouble();

    List<List<Node>> grid = new ArrayList<>();
    for (int y = 0; y < GRID_H; ++y) {
      List<Node> row = new ArrayList<>();
      for (int x = 0; x < GRID_W; ++x) {
        row.add(new Node());
      }
      grid.add(row);
    }
    for (int y = 0; y < GRID_H; ++y) {
      for (int x = 0; x < GRID_W; ++x) {
        grid.get(y).get(x).x = x;
        grid.get(y).get(x).y = y;
        grid.get(y).get(x).obstacle = data.get("grid").get(y).get(x).asBoolean();
      }
    }
    return new NavGrid(grid, nodeSizeMeters);
  }

  // auto BFSFirstFreeCell(std::vector<std::vector<Node>>& field, Point start_point)
  //     -> Node {
  //   int sx = start_point.x;
  //   int sy = start_point.y;
  //
  //   field[sy][sx].visited = true;
  //
  //   std::deque<Point> queue;
  //   queue.push_back(start_point);
  //
  //   std::vector<std::pair<int, int>> dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
  //                                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  //
  //   while (!queue.empty()) {
  //     const Point current_point = queue.front();
  //     queue.pop_front();
  //
  //     Node& current = field[current_point.y][current_point.x];
  //
  //     for (auto [dy, dx] : dirs) {
  //       int nx = current.x + dx;
  //       int ny = current.y + dy;
  //
  //       if (nx >= 0 && nx < (int)field[0].size() && ny >= 0 &&
  //           ny < (int)field.size()) {
  //         if (!field[ny][nx].visited) {
  //           field[ny][nx].visited = true;
  //           if (!field[ny][nx].obstacle) {
  //             return field[ny][nx];
  //           }
  //           queue.push_back({.x = (uint)nx, .y = (uint)ny});
  //         }
  //       }
  //     }
  //   }
  //
  //   return field[sy][sx];
  // }
  public static Node BFSFirstFreeCell(List<List<Node>> field, Point start_point) {
    int sx = start_point.x;
    int sy = start_point.y;

    field.get(sy).get(sx).visited = true;

    ArrayDeque<Point> queue = new ArrayDeque<>();
    queue.addLast(start_point);

    int[][] dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    while (!queue.isEmpty()) {
      final Point current_point = queue.pollFirst();

      Node current = field.get(current_point.y).get(current_point.x);

      for (int[] dir : dirs) {
        int dy = dir[0];
        int dx = dir[1];
        int nx = current.x + dx;
        int ny = current.y + dy;

        if (nx >= 0 && nx < (int) field.get(0).size() && ny >= 0 && ny < (int) field.size()) {
          if (!field.get(ny).get(nx).visited) {
            field.get(ny).get(nx).visited = true;
            if (!field.get(ny).get(nx).obstacle) {
              return field.get(ny).get(nx);
            }
            queue.addLast(new Point(nx, ny));
          }
        }
      }
    }

    return field.get(sy).get(sx);
  }

  // auto BFS(std::vector<std::vector<Node>>& field, Point start_point,
  //          Point end_point) -> std::vector<Node> {
  //
  //   int sx = start_point.x;
  //   int sy = start_point.y;
  //   if (field[sy][sx].obstacle) {
  //     Node adjusted = BFSFirstFreeCell(field, start_point);
  //     sx = adjusted.x;
  //     sy = adjusted.y;
  //   }
  //
  //   Node* start = &field[sy][sx];
  //   start_point.x = sx;
  //   start_point.y = sy;
  //   start->visited = true;
  //   start->cost = 0;
  //
  //   std::deque<Point> queue;
  //   queue.push_back(start_point);
  //
  //   Node end = {.x = end_point.x, .y = end_point.y};
  //
  //   bool path_completed = false;
  //
  //   std::vector<std::pair<int, int>> dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
  //                                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  //
  //   while (!path_completed && !queue.empty()) {
  //     const Point current_point = queue.front();
  //     int cx = current_point.x;
  //     int cy = current_point.y;
  //     Node& current = field[cy][cx];
  //     queue.pop_front();
  //
  //     for (std::pair<int, int> dir : dirs) {
  //       uint nx = current.x + dir.first;
  //       uint ny = current.y + dir.second;
  //
  //       if (nx >= 0 && nx < field[0].size()) {
  //         if (ny >= 0 && ny < field.size()) {
  //           if (!field[ny][nx].visited) {
  //             if (!field[ny][nx].obstacle) {
  //               Node* neighbor = &field[ny][nx];
  //               neighbor->x = nx;
  //               neighbor->y = ny;
  //               (abs(dir.first) == abs(dir.second))
  //                   ? neighbor->cost = current.cost + sqrt(2)
  //                   : neighbor->cost = current.cost + 1;
  //               neighbor->visited = true;
  //               neighbor->parent = &field[cy][cx];
  //
  //               queue.push_back({.x = nx, .y = ny});
  //               if (ny == end.y && nx == end.x) {
  //                 path_completed = true;
  //
  //                 break;
  //               }
  //             }
  //           }
  //         }
  //       }
  //     }
  //
  //     if (path_completed) {
  //       break;
  //     }
  //   }
  //
  //   if (!path_completed) {
  //     LOG(INFO) << "path couldn't be completed";
  //     return {};
  //   }
  //
  //   Node* rcurrent = &field[end.y][end.x];
  //   std::vector<Node> rpath = {};
  //   while (rcurrent != nullptr &&
  //          !(rcurrent->x == start->x && rcurrent->y == start->y)) {
  //     rpath.push_back(*rcurrent);
  //     rcurrent = rcurrent->parent;
  //     if (rcurrent != nullptr) {
  //       field[rcurrent->y][rcurrent->x].path = true;
  //     }
  //   }
  //   if (rcurrent != nullptr) {
  //     rpath.push_back(*rcurrent);
  //   }
  //   std::reverse(rpath.begin(), rpath.end());
  //
  //   std::cout << "BFS path (" << start_point.x << "," << start_point.y << ") -> ("
  //             << end_point.x << "," << end_point.y << "): ";
  //   for (const auto& node : rpath) {
  //     std::cout << "(" << node.x << "," << node.y << ") ";
  //   }
  //   std::cout << std::endl;
  //
  //   return rpath;
  // }
  //
  // NOTE: LOG(INFO) is a custom macro from src/utils/log.h in the source
  // repo and has no equivalent here; replaced with System.out.println as the
  // closest literal placeholder.
  public static List<Node> BFS(List<List<Node>> field, Point start_point, Point end_point) {
    int sx = start_point.x;
    int sy = start_point.y;
    if (field.get(sy).get(sx).obstacle) {
      Node adjusted = BFSFirstFreeCell(field, start_point);
      sx = adjusted.x;
      sy = adjusted.y;
    }

    Node start = field.get(sy).get(sx);
    start_point.x = sx;
    start_point.y = sy;
    start.visited = true;
    start.cost = 0;

    ArrayDeque<Point> queue = new ArrayDeque<>();
    queue.addLast(start_point);

    Node end = new Node();
    end.x = end_point.x;
    end.y = end_point.y;

    boolean path_completed = false;

    int[][] dirs = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    while (!path_completed && !queue.isEmpty()) {
      final Point current_point = queue.peekFirst();
      int cx = current_point.x;
      int cy = current_point.y;
      Node current = field.get(cy).get(cx);
      queue.pollFirst();

      for (int[] dir : dirs) {
        int nx = current.x + dir[0];
        int ny = current.y + dir[1];

        if (nx >= 0 && nx < field.get(0).size()) {
          if (ny >= 0 && ny < field.size()) {
            if (!field.get(ny).get(nx).visited) {
              if (!field.get(ny).get(nx).obstacle) {
                Node neighbor = field.get(ny).get(nx);
                neighbor.x = nx;
                neighbor.y = ny;
                if (Math.abs(dir[0]) == Math.abs(dir[1])) {
                  neighbor.cost = current.cost + Math.sqrt(2);
                } else {
                  neighbor.cost = current.cost + 1;
                }
                neighbor.visited = true;
                neighbor.parent = field.get(cy).get(cx);

                queue.addLast(new Point(nx, ny));
                if (ny == end.y && nx == end.x) {
                  path_completed = true;

                  break;
                }
              }
            }
          }
        }
      }

      if (path_completed) {
        break;
      }
    }

    if (!path_completed) {
      System.out.println("path couldn't be completed");
      return new ArrayList<>();
    }

    Node rcurrent = field.get(end.y).get(end.x);
    List<Node> rpath = new ArrayList<>();
    while (rcurrent != null && !(rcurrent.x == start.x && rcurrent.y == start.y)) {
      rpath.add(rcurrent);
      rcurrent = rcurrent.parent;
      if (rcurrent != null) {
        field.get(rcurrent.y).get(rcurrent.x).path = true;
      }
    }
    if (rcurrent != null) {
      rpath.add(rcurrent);
    }
    java.util.Collections.reverse(rpath);

    System.out.print(
        "BFS path ("
            + start_point.x
            + ","
            + start_point.y
            + ") -> ("
            + end_point.x
            + ","
            + end_point.y
            + "): ");
    for (Node node : rpath) {
      System.out.print("(" + node.x + "," + node.y + ") ");
    }
    System.out.println();

    return rpath;
  }
}
