//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.ACTIONS.IntakeAction;
//import org.firstinspires.ftc.teamcode.ACTIONS.ShootAction;
//import org.firstinspires.ftc.teamcode.ACTIONS.StopTransferAction;
//import org.firstinspires.ftc.teamcode.ACTIONS.TransferAction;
//import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
//import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Config
//@Autonomous(name = "Auto Blue Close 2")
//public class AutoBlueClose2 extends OpMode {
//
//    public enum SequenceState {
//        PRELOAD_SHOT,
//        TO_SPIKE_1,
//        RETURN_SHOT_1,
//        TO_SPIKE_2,
//        RETURN_SHOT_2,
//        TO_GATE_1,
//        RETURN_SHOT_3,
//        TO_GATE_2,
//        RETURN_SHOT_4,
//        TO_GATE_3,
//        RETURN_SHOT_5,
//        FINISHED
//    }
//
//    // --- Tunable parameters ---
//    public static double fireDurationSeconds          = 1.5;
//    public static double maxWaitForShotSeconds        = 2.5;
//    public static double gateWaitSeconds              = 1;
//    public static double headingReadyToleranceDeg     = 3.0;
//    public static boolean requireHeadingReadyForShot  = false;
//    public static boolean forceFeedAfterWaitTimeout   = true;
//
//    // Shooting zone triangle 1 (upper field)
//    public static double shootZoneTriangleOneAXInches = 72;
//    public static double shootZoneTriangleOneAYInches = 72;
//    public static double shootZoneTriangleOneBXInches = 0.0;
//    public static double shootZoneTriangleOneBYInches = 144;
//    public static double shootZoneTriangleOneCXInches = 144;
//    public static double shootZoneTriangleOneCYInches = 144;
//
//    // Shooting zone triangle 2 (lower field)
//    public static double shootZoneTriangleTwoAXInches = 72;
//    public static double shootZoneTriangleTwoAYInches = 23;
//    public static double shootZoneTriangleTwoBXInches = 95;
//    public static double shootZoneTriangleTwoBYInches = 0.0;
//    public static double shootZoneTriangleTwoCXInches = 49;
//    public static double shootZoneTriangleTwoCYInches = 0.0;
//
//    private static final double INCHES_TO_METERS = 0.0254;
//
//    // --- Poses (from AutoCloseBlue1_1.pp) ---
//
//    // Start: (37.348, 136.135), heading linear 90 -> 180 deg
//    private final Pose startPose = new Pose(37.34831836370614, 136.13465551339004, Math.toRadians(90));
//
//    // StartToShoot endpoint — heading linear 180 -> 134.771
//    private final Pose shootPose = new Pose(59.66467065868264, 83.85628742514972, Math.toRadians(134.771));
//
//    // Gate3ToShoot has a different endpoint
//    private final Pose shootPoseFinal = new Pose(68.44226538685398, 97.37852795233258, Math.toRadians(134.771));
//
//    // Spike 1 — ShootToSpike1 endpoint, heading linear 134.771 -> 180
//    private final Pose spike1Pose        = new Pose(16.25205930807249, 84.36573311367383, Math.toRadians(180));
//    private final Pose spike1ControlPose = new Pose(53.66676696031331, 79.04511241109216);
//
//    // Spike 2 — ShootToSpike2 endpoint, heading linear 134.771 -> 180
//    // Control points from .pp (note order matches file: first cp is closer to start)
//    private final Pose spike2Pose          = new Pose(13.996705107084011, 59.25535420098848, Math.toRadians(180));
//    private final Pose spike2Control1Pose  = new Pose(41.60828261105465,  55.796347995935626);
//    private final Pose spike2Control2Pose  = new Pose(52.04650336888003,  58.06652921504601);
//
//    // Spike2ToShoot control points, heading linear 180 -> 134.771
//    private final Pose spike2ReturnControl1Pose = new Pose(49.082747190955814, 59.829296925095434);
//    private final Pose spike2ReturnControl2Pose = new Pose(53.57203878897888,  74.55582081306909);
//
//    // Gate (shared endpoint for all 3 gate visits), heading linear -> 147
//    private final Pose gatePose        = new Pose(12.07578253706755, 60.0, Math.toRadians(147));
//    private final Pose gateControlPose = new Pose(36.428710947133744, 64.18349791356334);
//
//    // Gate1ToShoot control point (from .pp)
//    private final Pose gate1ReturnControlPose = new Pose(29.683817596662212, 42.501775544679276);
//
//    // Gate2ToShoot control point (from .pp — different from gate 1)
//    private final Pose gate2ReturnControlPose = new Pose(32.591268533772656, 43.511339758703365);
//
//    // Gate3ToShoot control point (from .pp)
//    private final Pose gate3ReturnControlPose = new Pose(39.47100494233938, 60.35483234519431);
//
//    // --- Subsystems ---
//    private Follower           follower;
//    private FlyWheelSubsystem  flyWheelSubsystem;
//    private IntakeSubsystem    intakeSubsystem;
//    private TransferSubsystem  transferSubsystem;
//    private ShooterCalculator  shooterCalculator;
//
//    // --- Actions ---
//    private ShootAction       shootAction;
//    private TransferAction    transferAction;
//    private StopTransferAction stopTransferAction;
//    private IntakeAction      intakeAction;
//
//    // --- Timers ---
//    private final ElapsedTime segmentTimer  = new ElapsedTime();
//    private final ElapsedTime shotTimer     = new ElapsedTime();
//    private final ElapsedTime holdTimer     = new ElapsedTime();
//    private final ElapsedTime pathEndTimer  = new ElapsedTime();
//
//    // --- State machine ---
//    private SequenceState sequenceState;
//    private SequenceState nextSequenceState;
//
//    private boolean segmentStarted;
//    private boolean shootDuringPath;
//    private boolean intakeDuringPath;
//    private boolean shotTriggered;
//    private boolean shotFinished;
//    private boolean waitAtEndOfPath;
//    private boolean holdStarted;
//    private boolean pathEndWaitStarted;
//
//    private PathChain currentPath;
//
//    // --- Paths ---
//    private PathChain driveStartToShoot;
//    private PathChain driveShootToSpike1;
//    private PathChain driveSpike1ToShoot;
//    private PathChain driveShootToSpike2;
//    private PathChain driveSpike2ToShoot;
//    private PathChain driveShootToGate1;
//    private PathChain driveGate1ToShoot;
//    private PathChain driveShootToGate2;
//    private PathChain driveGate2ToShoot;
//    private PathChain driveShootToGate3;
//    private PathChain driveGate3ToShoot;
//
//    @Override
//    public void init() {
//        flyWheelSubsystem  = new FlyWheelSubsystem(hardwareMap);
//        intakeSubsystem    = new IntakeSubsystem(hardwareMap);
//        transferSubsystem  = new TransferSubsystem(hardwareMap);
//        shooterCalculator  = new ShooterCalculator(ShooterCalculator.GoalTarget.BLUE);
//
//        shootAction        = new ShootAction(flyWheelSubsystem);
//        transferAction     = new TransferAction(transferSubsystem, intakeSubsystem);
//        stopTransferAction = new StopTransferAction(transferSubsystem, intakeSubsystem);
//        intakeAction       = new IntakeAction(intakeSubsystem);
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        buildPaths();
//        setSequenceState(SequenceState.PRELOAD_SHOT);
//        stopTransferAction.run();
//    }
//
//    @Override
//    public void start() {
//        segmentStarted = false;
//        segmentTimer.reset();
//        shotTimer.reset();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        configureSequenceIfNeeded();
//        runCurrentSequence();
//
//        PoseMemory.lastPose = follower.getPose();
//
//        telemetry.addData("Sequence",       sequenceState);
//        telemetry.addData("In Shoot Zone",  inShootingZone(follower.getPose().getX(), follower.getPose().getY()));
//        telemetry.addData("Path Busy",      follower.isBusy());
//        telemetry.addData("Shot Triggered", shotTriggered);
//        telemetry.addData("Shot Finished",  shotFinished);
//        telemetry.addData("Pose X",         follower.getPose().getX());
//        telemetry.addData("Pose Y",         follower.getPose().getY());
//        telemetry.addData("Pose Heading",   follower.getPose().getHeading());
//        telemetry.addData("Segment Time",   segmentTimer.seconds());
//        telemetry.addData("Shot Time",      shotTimer.seconds());
//        telemetry.addData("Intake State",   intakeSubsystem.getState());
//        telemetry.addData("Ball Detected",  intakeSubsystem.isBallDetected());
//        telemetry.update();
//    }
//
//    private void buildPaths() {
//
//        // StartToShoot — BezierLine, heading linear 90 -> 134.771 deg
//        driveStartToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
//                .build();
//
//        // ShootToSpike1 — BezierCurve 1 control point, heading linear 134.771 -> 180 deg
//        driveShootToSpike1 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, spike1ControlPose, spike1Pose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Pose.getHeading())
//                .build();
//
//        // Spike1ToShoot — BezierLine, heading linear 180 -> 134.771 deg
//        driveSpike1ToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(spike1Pose, shootPose))
//                .setLinearHeadingInterpolation(spike1Pose.getHeading(), shootPose.getHeading())
//                .build();
//
//        // ShootToSpike2 — BezierCurve 2 control points, heading linear 134.771 -> 180 deg
//        driveShootToSpike2 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, spike2Control1Pose, spike2Control2Pose, spike2Pose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), spike2Pose.getHeading())
//                .build();
//
//        // Spike2ToShoot — BezierCurve 2 control points, heading linear 180 -> 134.771 deg
//        driveSpike2ToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(spike2Pose, spike2ReturnControl1Pose, spike2ReturnControl2Pose, shootPose))
//                .setLinearHeadingInterpolation(spike2Pose.getHeading(), shootPose.getHeading())
//                .build();
//
//        // ShootToGate1 — BezierCurve 1 control point, heading linear 134.771 -> 147 deg
//        driveShootToGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, gateControlPose, gatePose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
//                .build();
//
//        // Gate1ToShoot — BezierCurve 1 control point, heading linear 147 -> 134.771 deg
//        driveGate1ToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(gatePose, gate1ReturnControlPose, shootPose))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
//                .build();
//
//        // ShootToGate2 — BezierCurve 1 control point, heading linear 134.771 -> 147 deg
//        driveShootToGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, gateControlPose, gatePose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
//                .build();
//
//        // Gate2ToShoot — BezierCurve 1 control point, heading linear 147 -> 134.771 deg
//        driveGate2ToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(gatePose, gate2ReturnControlPose, shootPose))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
//                .build();
//
//        // ShootToGate3 — BezierCurve 1 control point, heading linear 134.771 -> 147 deg
//        driveShootToGate3 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, gateControlPose, gatePose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
//                .build();
//
//        // Gate3ToShoot — BezierCurve 1 control point, heading linear 147 -> 134.771 deg
//        // Note: ends at shootPoseFinal (68.44, 97.38), not the standard shootPose
//        driveGate3ToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(gatePose, gate3ReturnControlPose, shootPoseFinal))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPoseFinal.getHeading())
//                .build();
//    }
//
//    private void setSequenceState(SequenceState newState) {
//        sequenceState      = newState;
//        segmentStarted     = false;
//        segmentTimer.reset();
//        shotTimer.reset();
//        shotTriggered      = false;
//        shotFinished       = false;
//        waitAtEndOfPath    = false;
//        holdStarted        = false;
//        pathEndWaitStarted = false;
//    }
//
//    private void configureSequenceIfNeeded() {
//        if (segmentStarted) return;
//
//        shootDuringPath   = false;
//        intakeDuringPath  = false;
//        waitAtEndOfPath   = false;
//        currentPath       = null;
//        nextSequenceState = SequenceState.FINISHED;
//
//        switch (sequenceState) {
//            case PRELOAD_SHOT:
//                currentPath       = driveStartToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.TO_SPIKE_1;
//                break;
//            case TO_SPIKE_1:
//                currentPath       = driveShootToSpike1;
//                intakeDuringPath  = true;
//                nextSequenceState = SequenceState.RETURN_SHOT_1;
//                break;
//            case RETURN_SHOT_1:
//                currentPath       = driveSpike1ToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.TO_SPIKE_2;
//                break;
//            case TO_SPIKE_2:
//                currentPath       = driveShootToSpike2;
//                intakeDuringPath  = true;
//                nextSequenceState = SequenceState.RETURN_SHOT_2;
//                break;
//            case RETURN_SHOT_2:
//                currentPath       = driveSpike2ToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.TO_GATE_1;
//                break;
//            case TO_GATE_1:
//                currentPath       = driveShootToGate1;
//                intakeDuringPath  = true;
//                waitAtEndOfPath   = true;
//                nextSequenceState = SequenceState.RETURN_SHOT_3;
//                break;
//            case RETURN_SHOT_3:
//                currentPath       = driveGate1ToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.TO_GATE_2;
//                break;
//            case TO_GATE_2:
//                currentPath       = driveShootToGate2;
//                intakeDuringPath  = true;
//                waitAtEndOfPath   = true;
//                nextSequenceState = SequenceState.RETURN_SHOT_4;
//                break;
//            case RETURN_SHOT_4:
//                currentPath       = driveGate2ToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.TO_GATE_3;
//                break;
//            case TO_GATE_3:
//                currentPath       = driveShootToGate3;
//                intakeDuringPath  = true;
//                waitAtEndOfPath   = true;
//                nextSequenceState = SequenceState.RETURN_SHOT_5;
//                break;
//            case RETURN_SHOT_5:
//                currentPath       = driveGate3ToShoot;
//                shootDuringPath   = true;
//                nextSequenceState = SequenceState.FINISHED;
//                break;
//            case FINISHED:
//                transferSubsystem.Closed();
//                shootAction.stop();
//                intakeAction.stop();
//                shotFinished   = true;
//                segmentStarted = true;
//                return;
//        }
//
//        follower.followPath(currentPath, true);
//        segmentStarted     = true;
//        shotTriggered      = !shootDuringPath;
//        shotFinished       = !shootDuringPath;
//        holdStarted        = false;
//        pathEndWaitStarted = false;
//
//        segmentTimer.reset();
//        shotTimer.reset();
//        holdTimer.reset();
//        pathEndTimer.reset();
//
//        transferSubsystem.Closed();
//    }
//
//    private void runCurrentSequence() {
//        if (sequenceState == SequenceState.FINISHED) return;
//
//        Pose currentPose = follower.getPose();
//
//        double robotXMeters   = currentPose.getX() * INCHES_TO_METERS;
//        double robotYMeters   = currentPose.getY() * INCHES_TO_METERS;
//        double robotHeadingRad = currentPose.getHeading();
//
//        double robotVelocityXMetersPerSecond =
//                follower.getVelocity().dot(new Vector(1.0, 0.0)) * INCHES_TO_METERS;
//        double robotVelocityYMetersPerSecond =
//                follower.getVelocity().dot(new Vector(1.0, Math.PI / 2.0)) * INCHES_TO_METERS;
//
//        ShooterCalculator.ShotSolution shotSolution = shooterCalculator.calculateShotSolution(
//                robotXMeters,
//                robotYMeters,
//                robotHeadingRad,
//                robotVelocityXMetersPerSecond,
//                robotVelocityYMetersPerSecond
//        );
//
//        if (shootDuringPath) {
//            shootAction.update(shotSolution);
//
//            boolean insideShootingZone = inShootingZone(currentPose.getX(), currentPose.getY());
//            boolean readyToShoot       = flyWheelSubsystem.isAtSpeed(2300);
//            boolean pathDone           = !follower.isBusy();
//
//            boolean hasBall = intakeAction.hasBall() &&
//                    intakeSubsystem.getState() == IntakeSubsystem.IntakeState.BALL_HELD_ONE;
//
//            boolean zoneAndReady    = insideShootingZone && readyToShoot && hasBall;
//            boolean fallbackReady   = pathDone && readyToShoot && hasBall;
//
//            if (!shotTriggered && (zoneAndReady || fallbackReady)) {
//                transferAction.start();
//                shotTriggered      = true;
//                pathEndWaitStarted = false;
//                shotTimer.reset();
//            }
//
//            if (shotTriggered && !shotFinished && shotTimer.seconds() >= fireDurationSeconds) {
//                transferSubsystem.Closed();
//
//                if (!intakeDuringPath) {
//                    intakeSubsystem.off();
//                } else {
//                    if (intakeSubsystem.getState() != IntakeSubsystem.IntakeState.INTAKING) {
//                        intakeSubsystem.runAll();
//                    }
//                }
//
//                shotFinished = true;
//            }
//
//            if (!shotTriggered && pathDone && !pathEndWaitStarted) {
//                pathEndWaitStarted = true;
//                pathEndTimer.reset();
//            }
//
//            if (!shotTriggered && pathDone && pathEndWaitStarted
//                    && pathEndTimer.seconds() >= maxWaitForShotSeconds) {
//                shotFinished = true;
//                transferSubsystem.Closed();
//
//                if (intakeDuringPath) {
//                    intakeSubsystem.runAll();
//                } else {
//                    intakeSubsystem.off();
//                }
//            }
//        } else {
//            shootAction.idle();
//        }
//
//        if (intakeDuringPath && !shotTriggered) {
//            if (intakeSubsystem.getState() == IntakeSubsystem.IntakeState.OFF) {
//                intakeAction.startIntake();
//            }
//        }
//
//        if (!follower.isBusy()) {
//            if (waitAtEndOfPath && !holdStarted) {
//                holdStarted = true;
//                holdTimer.reset();
//            }
//        } else {
//            holdStarted        = false;
//            pathEndWaitStarted = false;
//            holdTimer.reset();
//            pathEndTimer.reset();
//        }
//
//        boolean holdComplete = !waitAtEndOfPath ||
//                (holdStarted && holdTimer.seconds() >= gateWaitSeconds);
//
//        if (!follower.isBusy() && shotFinished && holdComplete) {
//            setSequenceState(nextSequenceState);
//        }
//    }
//
//    // --- Shooting zone helpers ---
//
//    private double lineSideValue(double pointX, double pointY,
//                                 double lineStartX, double lineStartY,
//                                 double lineEndX,   double lineEndY) {
//        return (pointX - lineStartX) * (lineEndY - lineStartY)
//                - (pointY - lineStartY) * (lineEndX - lineStartX);
//    }
//
//    private boolean isInsideTriangle(double pointX, double pointY,
//                                     double ax, double ay,
//                                     double bx, double by,
//                                     double cx, double cy) {
//        double sideAB = lineSideValue(pointX, pointY, ax, ay, bx, by);
//        double sideBC = lineSideValue(pointX, pointY, bx, by, cx, cy);
//        double sideCA = lineSideValue(pointX, pointY, cx, cy, ax, ay);
//
//        boolean hasNegative = sideAB < 0 || sideBC < 0 || sideCA < 0;
//        boolean hasPositive = sideAB > 0 || sideBC > 0 || sideCA > 0;
//
//        return !(hasNegative && hasPositive);
//    }
//
//    private boolean inShootingZone(double robotXInches, double robotYInches) {
//        boolean inTriangleOne = isInsideTriangle(
//                robotXInches, robotYInches,
//                shootZoneTriangleOneAXInches, shootZoneTriangleOneAYInches,
//                shootZoneTriangleOneBXInches, shootZoneTriangleOneBYInches,
//                shootZoneTriangleOneCXInches, shootZoneTriangleOneCYInches
//        );
//        boolean inTriangleTwo = isInsideTriangle(
//                robotXInches, robotYInches,
//                shootZoneTriangleTwoAXInches, shootZoneTriangleTwoAYInches,
//                shootZoneTriangleTwoBXInches, shootZoneTriangleTwoBYInches,
//                shootZoneTriangleTwoCXInches, shootZoneTriangleTwoCYInches
//        );
//        return inTriangleOne || inTriangleTwo;
//    }
//}