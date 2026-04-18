package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import java.util.Optional;

public class AutoHandler {

    private static final double LOOKAHEAD_SECONDS = 0.2;
    private static final double RECOVERY_TARGET_TOLERANCE_METERS = 0.20;
    private static final double COLLISION_DETECTION_TOLERANCE_METERS = 0.15;

    private enum RecoveryState {
        IDLE,
        ALIGNING,
        RESUME
    }

    private final RobotContainer robotContainer;
    private final Timer autoTimer = new Timer();

    private RecoveryState recoveryState = RecoveryState.IDLE;
    private PausableAuto activeAuto;
    private RobotConfig robotConfig;

    private boolean recoveryRequested = false;
    private Command alignCommand;
    private Pose2d recoveryTargetPose;
    private String lastRecoverPathName = "";

    private PathPlannerTrajectory cachedTrajectory;
    private String cachedTrajectoryPathName = "";

    private String currentPathName = "";
    private double currentPathStartTime = 0.0;

    private double recoveryTargetTime = 0.0;

    public AutoHandler(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception ex) {
            DriverStation.reportWarning(
                    "Auto recovery disabled: failed loading PathPlanner RobotConfig", false);
            robotConfig = null;
        }
    }

    public void onAutoStart(PausableAuto auto) {
        activeAuto = auto;
        recoveryRequested = false;
        recoveryState = RecoveryState.IDLE;
        alignCommand = null;
        recoveryTargetPose = null;
        lastRecoverPathName = "";
        cachedTrajectory = null;
        cachedTrajectoryPathName = "";

        currentPathName = "";
        currentPathStartTime = 0.0;

        autoTimer.reset();
        autoTimer.start();
    }

    public void onAutoEnd() {
        if (alignCommand != null && alignCommand.isScheduled()) {
            alignCommand.cancel();
        }

        activeAuto = null;
        recoveryRequested = false;
        recoveryState = RecoveryState.IDLE;
        alignCommand = null;
        cachedTrajectory = null;
        cachedTrajectoryPathName = "";
        recoveryTargetPose = null;
        lastRecoverPathName = "";

        autoTimer.stop();
        autoTimer.reset();
    }

    public void requestRecovery() {
        if (activeAuto != null && activeAuto.isScheduled()) {
            recoveryRequested = true;
        }
    }

    public double getElapsedAutoSeconds() {
        return autoTimer.get();
    }

    public Optional<Pose2d> getRecoveryTargetPose() {
        return Optional.ofNullable(recoveryTargetPose);
    }

    public boolean isRecovering() {
        return recoveryState != RecoveryState.IDLE;
    }

    private void clearRecovery(boolean unpauseAuto) {
        if (alignCommand != null && alignCommand.isScheduled()) {
            alignCommand.cancel();
        }
        alignCommand = null;

        if (unpauseAuto && activeAuto != null) {
            activeAuto.setPaused(false);
        }

        recoveryRequested = false;
        recoveryState = RecoveryState.IDLE;
    }

    private PathPlannerTrajectory buildTrajectoryForCurrentPath(String pathName) throws Exception {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return path.getIdealTrajectory(robotConfig).orElseThrow();
    }

    public void periodic() {
        if (activeAuto == null || !activeAuto.isScheduled()) {
            return;
        }

        String newPathName = com.pathplanner.lib.commands.PathPlannerAuto.currentPathName;

        if (newPathName != null && !newPathName.equals(currentPathName)) {
            currentPathName = newPathName;
            currentPathStartTime = autoTimer.get();
            cachedTrajectory = null;
            cachedTrajectoryPathName = "";
        }

        if (currentPathName == null || currentPathName.isBlank()) {
            return;
        }

        try {
            PathPlannerTrajectory trajectory = cachedTrajectory;
            if (trajectory == null || !cachedTrajectoryPathName.equals(currentPathName)) {
                trajectory = buildTrajectoryForCurrentPath(currentPathName);
                cachedTrajectory = trajectory;
                cachedTrajectoryPathName = currentPathName;
            }

            Pose2d currentPose = robotContainer.drivetrain.getState().Pose;

            double timeOnPath = autoTimer.get() - currentPathStartTime;

            double sampleTime = MathUtil.clamp(
                    timeOnPath + LOOKAHEAD_SECONDS,
                    0.0,
                    trajectory.getTotalTimeSeconds());

            PathPlannerTrajectoryState sampledState = trajectory.sample(sampleTime);
            if (AutoBuilder.shouldFlip()) {
                sampledState = sampledState.flip();
            }

            double translationError
                    = currentPose.getTranslation().getDistance(sampledState.pose.getTranslation());

            if (recoveryState == RecoveryState.IDLE && !activeAuto.isPaused()) {
                if (translationError > COLLISION_DETECTION_TOLERANCE_METERS) {
                    recoveryRequested = true;
                }
            }

            if (recoveryState == RecoveryState.IDLE) {
                if (!recoveryRequested || robotConfig == null) {
                    return;
                }

                activeAuto.setPaused(true);

                recoveryTargetTime = timeOnPath;

                double recoverySampleTime = MathUtil.clamp(
                        recoveryTargetTime + LOOKAHEAD_SECONDS,
                        0.0,
                        trajectory.getTotalTimeSeconds());

                PathPlannerTrajectoryState targetState = trajectory.sample(recoverySampleTime);
                if (AutoBuilder.shouldFlip()) {
                    targetState = targetState.flip();
                }

                recoveryTargetPose = targetState.pose;
                lastRecoverPathName = currentPathName;

                if (translationError <= RECOVERY_TARGET_TOLERANCE_METERS) {
                    recoveryState = RecoveryState.RESUME;
                    return;
                }

                alignCommand
                        = AutoBuilder.pathfindToPose(
                                recoveryTargetPose,
                                trajectory.getConstraints(),
                                targetState.linearVelocity);

                CommandScheduler.getInstance().schedule(alignCommand);
                recoveryState = RecoveryState.ALIGNING;
                return;
            }

            if (recoveryState == RecoveryState.ALIGNING) {
                if (alignCommand == null || !alignCommand.isScheduled()) {
                    recoveryRequested = false;
                    recoveryState = RecoveryState.RESUME;
                }
                return;
            }

            if (recoveryState == RecoveryState.RESUME) {
                clearRecovery(true);
            }

        } catch (Exception e) {
            DriverStation.reportWarning("Auto recovery error: " + e.getMessage(), false);
            clearRecovery(true);
        }
    }
}
