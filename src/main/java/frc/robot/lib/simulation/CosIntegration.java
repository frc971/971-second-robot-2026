package frc.robot.lib.simulation;

import com.fasterxml.jackson.databind.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.nio.file.*;
import java.util.*;

/** Opt-in headless tandem harness. All pickup and collision verdicts live here. */
public final class CosIntegration {
  private static final ObjectMapper JSON = new ObjectMapper();
  private static JsonNode layout;
  private static CosIntegration instance;
  private final boolean capture = "capture-james".equals(layout().path("mode").asText());
  private final boolean frozen = layout().path("mode").asText().startsWith("frozen-");
  private long benchmarkStarted;
  private int benchmarkPickups;
  public static boolean frozenBenchmark() { return configured() && layout().path("mode").asText().startsWith("frozen-"); }
  private final List<Object> secondPassPoses = new ArrayList<>();
  private boolean recording;
  private Map<String,Object> snapshot;
  public static void segmentStarted(String label) {
    if (instance == null) return;
    if (instance.frozen && (label.endsWith(" F_Normal") || label.endsWith(" dynamic:unet"))) {
      instance.benchmarkStarted = NetworkTablesJNI.now();
      instance.benchmarkPickups = FuelSim.getInstance().getCollected();
    }
    if (!instance.capture) return;
    try { instance.captureSegment(label); } catch (Exception e) { throw new IllegalStateException(e); }
  }
  private Map<String,Object> poseDocument() {
    var p = drive.integrationPose();
    return Map.of("translation_m", new double[]{p.getX(), p.getY()}, "rotation_rad", p.getRotation().getRadians());
  }
  private void captureSegment(String label) throws Exception {
    if (label.equals("3/5 F_Normal")) {
      snapshot = new LinkedHashMap<>();
      snapshot.put("source", "James before first F_Normal (second collection pass)");
      snapshot.put("red", layout.get("red").asBoolean()); snapshot.put("mirror", layout.get("mirror").asBoolean());
      snapshot.put("pose", poseDocument()); snapshot.put("balls", FuelSim.getInstance().getFuelPositions());
      snapshot.put("captured_at_seconds", (NetworkTablesJNI.now() - epoch) / 1e6);
      snapshot.put("opening_pickups", FuelSim.getInstance().getCollected());
      Files.writeString(Path.of(System.getenv("COS_INTEGRATION_RESULT")).resolveSibling("frozen-layout.json"), JSON.writeValueAsString(snapshot));
      recording = true;
    } else if (recording) {
      recording = false;
      Files.writeString(Path.of(System.getenv("COS_INTEGRATION_RESULT")).resolveSibling("james-second-pass-poses.json"), JSON.writeValueAsString(Map.of("poses", secondPassPoses)));
      finish(true, "James second-pass layout and reference trajectory captured");
    }
  }
  private final CommandSwerveDrivetrain drive;
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final StringPublisher scene = nt.getStringTopic("/Sim/COSIntegration/Scene").publish();
  private final StringSubscriber ready = nt.getStringTopic("/Sim/COSIntegration/Ready").subscribe("");
  private final BooleanSubscriber active = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Auto/UNetCollect/Active").subscribe(false);
  private final DoubleArraySubscriber goals = nt.getDoubleArrayTopic("/COS/BallVelocityGoal").subscribe(new double[0]);
  private boolean freshGoal;
  private long sequence, epoch, initialized = System.nanoTime();
  private int at20 = -1, sweepStart = -1;
  private boolean sweepPickup, scheduled, enabling;
  private int contactsAtStart;
  public void prepareAuto() {
    if (!frozen) return;
    var p = layout.get("pose");
    drive.resetPose(new edu.wpi.first.math.geometry.Pose2d(p.get("translation_m").get(0).asDouble(),
        p.get("translation_m").get(1).asDouble(), new edu.wpi.first.math.geometry.Rotation2d(p.get("rotation_rad").asDouble())));
  }
  public void autoStarted() { epoch = NetworkTablesJNI.now(); contactsAtStart = drive.integrationContacts(); }
  public static boolean configured() { return System.getenv("COS_INTEGRATION_LAYOUT") != null; }
  private static JsonNode layout() {
    try {
      if (layout == null) {
        byte[] bytes = Files.readAllBytes(Path.of(System.getenv("COS_INTEGRATION_LAYOUT")));
        String digest = HexFormat.of().formatHex(java.security.MessageDigest.getInstance("SHA-256").digest(bytes));
        if (!digest.equals(System.getenv("COS_INTEGRATION_SHA256"))) throw new IllegalArgumentException("Layout hash mismatch");
        layout = JSON.readTree(bytes);
      }
      return layout;
    } catch (Exception e) { throw new IllegalStateException(e); }
  }
  public static void spawn(FuelSim fuel) {
    spawn(fuel, layout());
  }
  public static void spawn(FuelSim fuel, JsonNode document) {
    for (var p : document.get("balls"))
      fuel.spawnFuel(new Translation3d(p.get(0).asDouble(), p.get(1).asDouble(), p.get(2).asDouble()), new Translation3d());
    fuel.setFrozen(document.path("mode").asText().startsWith("frozen-"));
  }
  public CosIntegration(CommandSwerveDrivetrain drive) {
    this.drive = drive; instance = this;
    layout();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(layout.get("red").asBoolean() ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    nt.getTable("SmartDashboard").getSubTable("Auto Mode").getEntry("selected")
      .setString((capture ? "James" : frozen ? (layout.path("mode").asText().equals("frozen-james") ? "COS second pass James" : "COS second pass UNet") : "Dynamic real UNet") + (layout.get("mirror").asBoolean() ? " (Mirrored)" : ""));
    var fuel = FuelSim.getInstance(); fuel.clearFuel(); spawn(fuel);
  }
  public void periodic(Command command) {
    try {
      long now = NetworkTablesJNI.now();
      if (!enabling && ((capture && System.nanoTime() - initialized > 2000000000L) || ready.get().equals(layout.get("run").asText() + ":" + layout.get("sha256").asText()))) {
        enabling = true;
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
      }
      if (recording) secondPassPoses.add(poseDocument());
      var pose = drive.integrationPose();
      var value = new LinkedHashMap<String,Object>();
      value.put("run", layout.get("run").asText()); value.put("sha256", layout.get("sha256").asText());
      value.put("sequence", sequence++); value.put("timestamp_us", now); value.put("enabled", epoch != 0);
      value.put("pose", Map.of("translation_m", new double[] {pose.getX(), pose.getY()}, "rotation_rad", pose.getRotation().getRadians()));
      value.put("balls", FuelSim.getInstance().getFuelPositions());
      value.put("benchmark_active", frozen && benchmarkStarted != 0);
      scene.set(JSON.writeValueAsString(value)); nt.flush();
      int collected = FuelSim.getInstance().getCollected();
      // This logger topic is also used by the existing dashboard.
      if (active.get()) {
        var goal = goals.getAtomic();
        if (goal.value.length == 10 && goal.value[5] != 0 && goal.timestamp >= epoch
            && now >= goal.timestamp && now - goal.timestamp <= 250000) freshGoal = true; if (sweepStart < 0) sweepStart = collected; if (collected > sweepStart && now - epoch <= 20000000) sweepPickup = true; }
      if (epoch != 0 && now - epoch >= 20000000 && at20 < 0) at20 = collected;
      if (command != null && command.isScheduled()) scheduled = true;
      if (epoch != 0 && drive.integrationContacts() != contactsAtStart) finish(false, "field collision");
      if (epoch == 0 && System.nanoTime() - initialized > 60000000000L) finish(false, "COS readiness timeout");
      if (epoch != 0 && now - epoch > 60000000) finish(false, "auto timeout");
      if (frozen && scheduled && !command.isScheduled()) {
        finish(benchmarkStarted != 0 && (!layout.path("mode").asText().equals("frozen-unet") || freshGoal), "second pass complete");
      }
      if (!frozen && at20 >= 0 && scheduled && !command.isScheduled()) finish(sweepPickup && freshGoal, !freshGoal ? "no fresh dynamic goals" : sweepPickup ? "complete" : "no dynamic pickups");
    } catch (Exception e) { throw new IllegalStateException(e); }
  }
  private void finish(boolean pass, String reason) throws Exception {
    DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
    var report = new LinkedHashMap<String,Object>();
    report.put("pass", pass); report.put("reason", reason); report.put("run", layout.get("run").asText());
    if (frozen) {
      report.put("second_pass_pickups", FuelSim.getInstance().getCollected() - benchmarkPickups);
      report.put("second_pass_seconds", benchmarkStarted == 0 ? 0 : (NetworkTablesJNI.now() - benchmarkStarted) / 1e6);
      report.put("ball_count", layout.get("balls").size());
      report.put("layout_sha256", layout.get("sha256").asText());
    } else if (at20 >= 0) report.put("pickups_at_20", at20);
    report.put("elapsed_seconds", epoch == 0 ? 0 : (NetworkTablesJNI.now() - epoch) / 1e6);
    report.put("pose", drive.integrationPose().toString());
    String result = JSON.writeValueAsString(report);
    Files.writeString(Path.of(System.getenv("COS_INTEGRATION_RESULT")), result);
    System.out.println(result); System.exit(pass ? 0 : 1);
  }
}
