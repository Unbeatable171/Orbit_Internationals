package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.Turrettt;

@Config
@Autonomous(name = "Auto Blue Close")
public class AutoBlueClose extends OpMode {

    public enum SequenceState {
        PRELOAD_SHOT,
        TO_SPIKE_1,
        RETURN_SHOT_1,
        TO_SPIKE_2,
        RETURN_SHOT_2,
        TO_GATE_1,
        RETURN_SHOT_3,
        TO_GATE_2,
        RETURN_SHOT_4,
        TO_GATE_3,
        RETURN_SHOT_5,
        TO_GATE_4,
        RETURN_SHOT_6,
        FINISHED
    }

    // -------------------------------------------------------------------------
    // Tunable parameters (tweakable via FTC Dashboard)
    // -------------------------------------------------------------------------
    public static double fireDurationSeconds   = 0.9;
    public static double maxWaitForShotSeconds = 2.5;
    public static double gateWaitSeconds       = 1.2;
    public static double rpmTolerance          = 125.0;

    // Preset 1 — preload shot from shootPose
    public static int    targetRPM1    = 2375;
    public static double hoodAngle1Deg = 67.0;

    // Preset 2 — all subsequent shots from shootPose2
    public static int    targetRPM2    = 2500;
    public static double hoodAngle2Deg = 65.0;

    // -------------------------------------------------------------------------
    // Goal pose — pulled from CONSTANTS so it stays in sync with the rest of
    // the codebase. Never hardcoded here.
    // -------------------------------------------------------------------------
    private Pose getBlueGoalPose() {
        return new Pose(
                CONSTANTS.blueGoalXInches - 8,
                CONSTANTS.blueGoalYInches
        );
    }

    // -------------------------------------------------------------------------
    // Poses
    // -------------------------------------------------------------------------
    private final Pose startPose                = new Pose(37.34831836370614, 136.13465551339004, Math.toRadians(90));
    private final Pose shootPose                = new Pose(58.6, 83.85628742514972, Math.toRadians(130));
    private final Pose shootPose2               = new Pose(59.5, 80, Math.toRadians(170));
    private final Pose spike1Pose               = new Pose(28, 86.39819893649711, Math.toRadians(180));
    private final Pose spike1ControlPose        = new Pose(48, 74.2);
    private final Pose spike2Pose               = new Pose(22.5, 62, Math.toRadians(180));
    private final Pose spike2Control1Pose       = new Pose(70, 50);
    private final Pose spike2ReturnControl1Pose = new Pose(85, 83);
    private final Pose gatePose                 = new Pose(8.2, 61, Math.toRadians(145));
    private final Pose gatemiddlePose           = new Pose(13.2,67, Math.toRadians(180));
    private final Pose gateControlPose          = new Pose(41.428710947133744, 64.18349791356334);
    private final Pose gateReturnControlPose    = new Pose(8, 50.65863141524106);
    private final Pose leavePose                = new Pose(40, 56.7, Math.toRadians(145));

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------
    private Follower          follower;
    private FlyWheelSubsystem flyWheelSubsystem;
    private IntakeSubsystem   intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private Turrettt          turret;

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime shotTimer    = new ElapsedTime();
    private final ElapsedTime holdTimer    = new ElapsedTime();
    private final ElapsedTime pathEndTimer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    private SequenceState sequenceState;
    private SequenceState nextSequenceState;
    private boolean segmentStarted;
    private boolean shootDuringPath;
    private boolean intakeDuringPath;
    private boolean shotTriggered;
    private boolean shotFinished;
    private boolean waitAtEndOfPath;
    private boolean holdStarted;
    private boolean pathEndWaitStarted;
    private PathChain currentPath;

    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------
    private PathChain driveStartToShoot;
    private PathChain driveShootToSpike1;
    private PathChain driveSpike1ToShoot;
    private PathChain driveShootToSpike2;
    private PathChain driveSpike2ToShoot;
    private PathChain driveShootToGate1;
    private PathChain driveGate1ToShoot;
    private PathChain driveShootToGate2;
    private PathChain driveGate2ToShoot;
    private PathChain driveShootToGate3;
    private PathChain driveGate3ToShoot;
    private PathChain driveShootToGate4;
    private PathChain driveGate4ToShoot;

    // =========================================================================
    // OpMode lifecycle
    // =========================================================================

    @Override
    public void init() {
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);
        intakeSubsystem   = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        turret            = new Turrettt(hardwareMap);

        follower = Pedropathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        setSequenceState(SequenceState.PRELOAD_SHOT);
        stopTransfer();
        stopIntake();

        // Park turret at safe centre while waiting for start
        turret.commandPosition(Turrettt.safeMiddle);
    }

    @Override
    public void start() {
        segmentStarted = false;
        segmentTimer.reset();
        shotTimer.reset();
    }

    @Override
    public void loop() {
        // 1. Drive update
        follower.update();

        // 2. Turret update — uses follower.getPose() directly, no Localization dependency
        updateTurret();

        // 3. Sequence logic
        configureSequenceIfNeeded();
        runCurrentSequence();

        // 4. Telemetry
        telemetry.addData("Sequence",        sequenceState);
        telemetry.addData("Path Busy",       follower.isBusy());
        telemetry.addData("Shot Triggered",  shotTriggered);
        telemetry.addData("Shot Finished",   shotFinished);
        telemetry.addData("Pose X",          follower.getPose().getX());
        telemetry.addData("Pose Y",          follower.getPose().getY());
        telemetry.addData("Pose Heading°",   Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Segment Time",    segmentTimer.seconds());
        telemetry.addData("Shot Time",       shotTimer.seconds());
        telemetry.addData("Left RPM",        flyWheelSubsystem.getCurrentRPMLeft());
        telemetry.addData("Right RPM",       flyWheelSubsystem.getCurrentRPMRight());
        telemetry.addData("Turret Pos",      turret.getServoPosition());
        telemetry.addData("Turret Target°",  turret.getCurrentTargetHeadingDegrees());
        telemetry.addData("Turret Routing",  turret.isRoutingThroughMiddle());
        telemetry.addData("Turret OnTarget", isTurretOnTarget());
        telemetry.update();
    }

    // =========================================================================
    // Turret management
    // =========================================================================

    /**
     * Called every loop().
     *
     * During shooting states: calculate the heading from the current robot pose
     * (from follower, not Localization) to the blue goal, convert to a servo
     * position, and command the turret there via setPosition() which applies
     * the same slew-rate limiting and wrap-routing as periodic() would.
     *
     * During travel/intake states: park at safeMiddle to stay out of the way.
     */
    private void updateTurret() {
        if (isTurretActiveState()) {
            Pose robotPose = follower.getPose();

            double targetHeadingRad = turret.calculateTargetHeading(robotPose, getBlueGoalPose());
            double targetServoPos   = turret.headingToTurretPos(targetHeadingRad);

            turret.setPosition(targetServoPos);
        } else {
            turret.commandPosition(Turrettt.safeMiddle);
        }
    }

    /**
     * Returns true for every state where the turret should actively track the
     * goal. Starts tracking on PRELOAD_SHOT so the turret is already settled
     * when we reach the shoot pose.
     */
    private boolean isTurretActiveState() {
        switch (sequenceState) {
            case PRELOAD_SHOT:
            case RETURN_SHOT_1:
            case RETURN_SHOT_2:
            case RETURN_SHOT_3:
            case RETURN_SHOT_4:
            case RETURN_SHOT_5:
            case RETURN_SHOT_6:
                return true;
            default:
                return false;
        }
    }

    // =========================================================================
    // Sequence configuration
    // =========================================================================

    private void setSequenceState(SequenceState newState) {
        sequenceState      = newState;
        segmentStarted     = false;
        segmentTimer.reset();
        shotTimer.reset();
        shotTriggered      = false;
        shotFinished       = false;
        waitAtEndOfPath    = false;
        holdStarted        = false;
        pathEndWaitStarted = false;
    }

    private void configureSequenceIfNeeded() {
        if (segmentStarted) return;

        shootDuringPath   = false;
        intakeDuringPath  = false;
        waitAtEndOfPath   = false;
        currentPath       = null;
        nextSequenceState = SequenceState.FINISHED;

        switch (sequenceState) {
            case PRELOAD_SHOT:
                currentPath       = driveStartToShoot;
                shootDuringPath   = true;
                nextSequenceState = SequenceState.TO_SPIKE_2;
                break;
            case TO_SPIKE_1:
                currentPath       = driveShootToSpike1;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.RETURN_SHOT_1;
                break;
            case RETURN_SHOT_1:
                currentPath       = driveSpike1ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.TO_GATE_2;
                break;
            case TO_SPIKE_2:
                currentPath       = driveShootToSpike2;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.RETURN_SHOT_2;
                break;
            case RETURN_SHOT_2:
                currentPath       = driveSpike2ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.TO_GATE_1;
                break;
            case TO_GATE_1:
                currentPath       = driveShootToGate1;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.RETURN_SHOT_3;
                break;
            case RETURN_SHOT_3:
                currentPath       = driveGate1ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.TO_SPIKE_1;
                break;
            case TO_GATE_2:
                currentPath       = driveShootToGate2;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.RETURN_SHOT_4;
                break;
            case RETURN_SHOT_4:
                currentPath       = driveGate2ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.TO_GATE_3;
                break;
            case TO_GATE_3:
                currentPath       = driveShootToGate3;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.RETURN_SHOT_5;
                break;
            case RETURN_SHOT_5:
                currentPath       = driveGate3ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.TO_GATE_4;
                break;
            case TO_GATE_4:
                currentPath       = driveShootToGate4;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case RETURN_SHOT_6:
                currentPath       = driveGate4ToShoot;
                shootDuringPath   = true;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case FINISHED:
                stopTransfer();
                flyWheelSubsystem.stop();
                stopIntake();
                turret.commandPosition(Turrettt.safeMiddle);
                shotFinished   = true;
                segmentStarted = true;
                return;
        }

        follower.followPath(currentPath, true);
        segmentStarted     = true;
        shotTriggered      = !shootDuringPath;
        shotFinished       = !shootDuringPath;
        holdStarted        = false;
        pathEndWaitStarted = false;
        segmentTimer.reset();
        shotTimer.reset();
        holdTimer.reset();
        pathEndTimer.reset();
        stopTransfer();

        if (intakeDuringPath) {
            startIntake();
        } else {
            stopIntake();
        }
    }

    // =========================================================================
    // Sequence execution
    // =========================================================================

    private void runCurrentSequence() {
        if (sequenceState == SequenceState.FINISHED) return;

        if (shootDuringPath) {
            int    activeRPM     = useShot2Preset() ? targetRPM2    : targetRPM1;
            double activeHoodDeg = useShot2Preset() ? hoodAngle2Deg : hoodAngle1Deg;

            flyWheelSubsystem.spinUp(activeRPM);
            flyWheelSubsystem.setHoodAngle(activeHoodDeg);

            boolean flywheelReady = isAtSpeed(activeRPM);
            boolean pathDone      = !follower.isBusy();

            if (pathDone && !shotTriggered && !shotFinished) {
                if (!pathEndWaitStarted) {
                    pathEndWaitStarted = true;
                    pathEndTimer.reset();
                }

                if (flywheelReady && isTurretOnTarget()) {
                    // Both flywheel and turret ready — fire
                    startTransfer();
                    shotTriggered = true;
                    shotTimer.reset();
                } else if (pathEndTimer.seconds() >= maxWaitForShotSeconds) {
                    // Timeout — fire anyway rather than skip the ring
                    startTransfer();
                    shotTriggered = true;
                    shotTimer.reset();
                }
            }

            if (shotTriggered && !shotFinished && shotTimer.seconds() >= fireDurationSeconds) {
                stopTransfer();
                shotFinished = true;
            }

        } else {
            flyWheelSubsystem.idle();
        }

        if (intakeDuringPath && !shotTriggered) {
            startIntake();
        }

        if (!follower.isBusy()) {
            if (waitAtEndOfPath && !holdStarted) {
                holdStarted = true;
                holdTimer.reset();
            }
        } else {
            holdStarted        = false;
            pathEndWaitStarted = false;
            holdTimer.reset();
            pathEndTimer.reset();
        }

        boolean holdComplete = !waitAtEndOfPath
                || (holdStarted && holdTimer.seconds() >= gateWaitSeconds);

        if (!follower.isBusy() && shotFinished && holdComplete) {
            setSequenceState(nextSequenceState);
        }
    }

    // =========================================================================
    // Helper predicates
    // =========================================================================

    /** True for all shot states that use the second (higher-power) preset. */
    private boolean useShot2Preset() {
        switch (sequenceState) {
            case RETURN_SHOT_1:
            case RETURN_SHOT_2:
            case RETURN_SHOT_3:
            case RETURN_SHOT_4:
            case RETURN_SHOT_5:
                return true;
            default:
                return false;
        }
    }

    /** Both wheels within rpmTolerance of the target. */
    private boolean isAtSpeed(double targetRPM) {
        return Math.abs(flyWheelSubsystem.getCurrentRPMLeft()  - targetRPM) <= rpmTolerance
                && Math.abs(flyWheelSubsystem.getCurrentRPMRight() - targetRPM) <= rpmTolerance;
    }

    /**
     * True when the turret servo has settled within 0.01 of its final target
     * and is not mid-wrap-routing. Loosen the threshold to 0.02–0.03 if you
     * find the turret never fully settles before the timeout.
     */
    private boolean isTurretOnTarget() {
        return !turret.isRoutingThroughMiddle()
                && Math.abs(turret.getServoPosition() - turret.getFinalTargetPosition()) < 0.01;
    }

    // =========================================================================
    // Intake / transfer helpers
    // =========================================================================

    private void startIntake() {
        intakeSubsystem.runAll();
    }

    private void stopIntake() {
        intakeSubsystem.off();
    }

    private void startTransfer() {
        transferSubsystem.Open();
        intakeSubsystem.transfer();
    }

    private void stopTransfer() {
        transferSubsystem.Closed();
    }

    // =========================================================================
    // Path building
    // =========================================================================

    private void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, spike1ControlPose, spike1Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), spike1Pose.getHeading())
                .build();

        driveSpike1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pose, shootPose2))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), shootPose2.getHeading())
                .build();

        driveShootToSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, spike2Control1Pose, spike2Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), spike2Pose.getHeading())
                .build();

        driveSpike2ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(spike2Pose, spike2ReturnControl1Pose, shootPose2))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(), shootPose2.getHeading())
                .build();

        driveShootToGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gatemiddlePose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .build();

        driveShootToGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gatemiddlePose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .build();

        driveShootToGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gatemiddlePose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .build();

        driveShootToGate4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, leavePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), leavePose.getHeading())
                .build();

        driveGate4ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .build();
    }
}