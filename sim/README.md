# Simulation GUI Configuration

The JSON files in this folder provide a default configuration for WPILib's simulation GUI, including docked windows and keyboard mapping for both the driver and operator Xbox controllers. Open the `Simulation Key Mapping.html` file in a browser to see the default key mappings to both controllers.

To use this configuration, **copy the three JSON files to the root directory of the robot project:**

```bash
cp sim/simgui*.json .
```

## Visual COS collection check

Desktop simulation includes a synthetic COS publisher (no camera or UNet inference).
Run `./gradlew simulateJava`, connect AdvantageScope to `localhost`, and add a 2D field:

- Robot: `/Drive/Pose` (or `/AdvantageKit/RealOutputs/Drive/Pose`).
- Cluster markers: `/AdvantageKit/RealOutputs/Sim/COS/YellowCluster`; set their color to yellow.
- Current target: `/AdvantageKit/RealOutputs/Sim/COS/Target`.
- Alternatively, view the complete `SmartDashboard/COS Simulation` Field2d widget.

Select **Dynamic real UNet** in `SmartDashboard/Auto Mode` while disabled, then select
Autonomous in the simulation GUI (this also enables the robot). Watch the initial sweep and
shooting path, the approach to x = 6.6 m (blue coordinates) along the field edge,
then the inward move to y = 6.9 m and turn with the rear intake facing field center.
After the three-second COS sweep, the robot turns in the open neutral zone, enters
the trench lane at y = 7.42 m, and crosses to x = 3.1 m before the final shooting path. The simulated cluster has three target columns;
the target advances when the robot comes within 0.3 m of each column.
Use `Auto/UNetCollect/Active`, `Fresh`, and `ElapsedSeconds` under AdvantageKit's
`RealOutputs` to identify the collection window.

NetworkTables controls under `/Sim/COS`:

| Entry | Default | Purpose |
|---|---|---|
| `Enabled` | `true` | Publish synthetic goals every robot loop. Set false when using real COS; stopping updates also tests the 0.25-second freshness timeout. |
| `Detected` | `true` | Set false to publish an invalid detection and brake immediately during collection. |
| `ClusterX` | `6.2` | Cluster center X in blue field coordinates, meters. |
| `ClusterY` | `6.8` | Cluster center Y in blue field coordinates, meters. |

The cluster mirrors with the selected auto and flips for red alliance. Disable and
re-enable autonomous to reset target progression. Markers represent synthetic
observations, separate from FuelSim's collision/intake balls; this checks velocity
control and auto handoff rather than image recognition or ball pickup accuracy.

The simulation logs `Drive/Simulation/FieldContacts` under AdvantageKit's
`RealOutputs`. It counts actual dyn4j robot contacts with static field obstacles
since simulation startup; restart the simulation for a fresh zero count.

Run the physics regression with `./gradlew test --tests '*UnetTrenchClearanceTest'`.
It runs the full selected routine using MapleSim motor/wheel dynamics at 2 ms and
checks zero field contacts at every physics step, on both alliances, both mirror
settings, and with/without yellow detections. It also verifies the intake faces
field center before COS starts and that the collision listener detects a deliberate
wall contact. The harness uses MapleSim's motor controllers rather than CTRE's
controller threads; shooting/intake mechanism actions are stubbed. The full drive
sequence currently completes in approximately 28 seconds in this harness.
