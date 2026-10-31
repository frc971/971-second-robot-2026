package frc.robot.lib.pathing;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PathfindingTest {
  @Test
  void reusesPreloadedGridForRepeatedSearches(@TempDir Path directory) throws Exception {
    Path file = directory.resolve("navgrid.json");
    Files.writeString(
        file,
        "{\"nodeSizeMeters\":0.2,\"grid\":[[false,false,false],[false,true,false],[false,false,false]]}");
    Pathfinding.preloadNavGrid(file.toString());
    Files.delete(file);
    Pathfinding.preloadNavGrid(file.toString());
    NavGrid grid = Pathfinding.getNavGrid();
    assertEquals(0.2, grid.nodeSizeMeters);
    assertSame(grid, Pathfinding.getNavGrid());
    for (int[] endpoints : new int[][] {{0, 0, 2, 2}, {2, 2, 0, 0}, {1, 1, 2, 2}, {2, 2, 1, 1}}) {
      Point start = new Point(endpoints[0], endpoints[1]);
      List<Node> path = Pathfinding.BFS(grid.grid, start, new Point(endpoints[2], endpoints[3]));
      assertFalse(path.isEmpty());
      assertSame(grid.grid.get(start.y).get(start.x), path.get(0));
      assertNull(path.get(0).parent);
      assertTrue(path.stream().noneMatch(node -> node.obstacle));
      if (endpoints[2] != 1 || endpoints[3] != 1) {
        assertSame(grid.grid.get(endpoints[3]).get(endpoints[2]), path.get(path.size() - 1));
      }
    }
  }
}
