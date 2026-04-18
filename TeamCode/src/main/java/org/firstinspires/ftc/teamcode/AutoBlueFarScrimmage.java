package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ACTIONS.IntakeAction;
import org.firstinspires.ftc.teamcode.ACTIONS.ShootAction;
import org.firstinspires.ftc.teamcode.ACTIONS.StopTransferAction;
import org.firstinspires.ftc.teamcode.ACTIONS.TransferAction;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "Auto Blue Far Scrimmage")
public class AutoBlueFarScrimmage extends OpMode {

    public enum SequenceState {
        PRELOAD_SHOT,
        ToHumanZone,
        HumanToBack,
        TOSpike3,
        RETURN_SHOT_2,
        ToOverflow,
        RETURN_SHOT_3,
        TO_GATE_2,
        RETURN_SHOT_4,
        TO_GATE_3,
        RETURN_SHOT_5,
        FINISHED,
        BACK_TO_HUMAN,
        HUMAN_TO_SHOOT
    }

    // --- Tunable parameters ---
    public static double fireDurationSeconds   = 1;
    public static double maxWaitForShotSeconds = 2.5;
    public static double gateWaitSeconds       = 0.8;
    public static double transferOpenSeconds   = 0.2;
    public static double transferClosedSeconds = 0.3;
    public static int shotPulseCountTarget     = 3;

    public static double blueGoalXInches = 8;
    public static double blueGoalYInches = 136;
    // Fixed shooter values — tune hoodPosition for your 65 deg target
    public static int targetRPM1 = 3300;
    public static double hoodAngle1Deg = 50;   // degrees, not servo position

    public static int targetRPM2 = 2450;
    public static double hoodAngle2Deg = 60;   // degrees, not servo position

//    private boolean useShot2Preset() {
//        return sequenceState == SequenceState.RETURN_SHOT_1
//                || sequenceState == SequenceState.RETURN_SHOT_2
//                || sequenceState == SequenceState.RETURN_SHOT_3
//                || sequenceState == SequenceState.RETURN_SHOT_4
//                || sequenceState == SequenceState.RETURN_SHOT_5;
//    }


    private static final double INCHES_TO_METERS = 0.0254;

    private final Pose goalPoseBlue = new Pose(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches);

    // --- Poses ---
    private final Pose startPose  = new Pose(56,  8.9, Math.toRadians(90));
    private final Pose shootPose  = new Pose(56.11586314152411,   13.20373250388803, Math.toRadians(114.5));

    // Spike 1
    private final Pose humanZone = new Pose(25.5, 5, Math.toRadians(180));
    private final Pose backPose = new Pose(31.5,6,Math.toRadians(180));
    private final Pose humanzoneReturnControl = new Pose (26.274883359253494,17.184292379471227);
    private final Pose humanzoneReturnControl2 = new Pose (0,0);
//    private final Pose spike1ControlPose = new Pose(48, 74.2);
//    private final Pose shootPose2 = new Pose(61.1607182661587,73.05352407500875,Math.toRadians(132.41));

    // Spike 2
    private final Pose spike3pose = new Pose(25, 28, Math.toRadians(180));
    private final Pose controlSpike3 = new Pose(45.58357501607667,  24.54215162719411);
//    private final Pose spike2ReturnControl1Pose = new Pose(44.92984555509721, 58.9215464474854);

    // Gate
    private final Pose overflowPick = new Pose(16, 35.12498924965115, Math.toRadians(120));
    private final Pose overflowControlPose = new Pose(13.893564047419593, 2.0286688083403455);
//    private final Pose gateReturnControlPose = new Pose(7.444645799011532,  31.175260681273393);

    // --- Subsystems ---
    private Follower          follower;
    private FlyWheelSubsystem flyWheelSubsystem;
    private IntakeSubsystem   intakeSubsystem;
    private TransferSubsystem transferSubsystem;

    // --- Actions ---
    private ShootAction        shootAction;
    private TransferAction     transferAction;
    private StopTransferAction stopTransferAction;
    private IntakeAction       intakeAction;

    // --- Timers ---
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime shotTimer    = new ElapsedTime();
    private final ElapsedTime holdTimer    = new ElapsedTime();
    private final ElapsedTime pathEndTimer = new ElapsedTime();

    // --- State machine ---
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
    private boolean transferCurrentlyOpen;
    private int shotPulseCount;

    private PathChain currentPath;

    // --- Paths ---
    private PathChain driveStartToShoot;
    private PathChain driveShootToHuman;
    private PathChain driveHumanToBack;
    private PathChain driveShoottoSpike3;
    private PathChain driveSpike3ToShoot;
    private PathChain driveShootToOverflow;
    private PathChain driveOverflowToShoot;
    private PathChain driveShoottoOverflow2;
    private PathChain driveOverflowtoShoot2;
    private PathChain driveShootToOverflow3;
    private PathChain driveOverflow3ToShoot3;
    private PathChain driveHumanToShoot;
    private PathChain driveBackToHuman;

    @Override
    public void init() {
        flyWheelSubsystem  = new FlyWheelSubsystem(hardwareMap);
        intakeSubsystem    = new IntakeSubsystem(hardwareMap);
        transferSubsystem  = new TransferSubsystem(hardwareMap);

        shootAction        = new ShootAction(flyWheelSubsystem);
        transferAction     = new TransferAction(transferSubsystem, intakeSubsystem);
        stopTransferAction = new StopTransferAction(transferSubsystem, intakeSubsystem);
        intakeAction       = new IntakeAction(intakeSubsystem);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        setSequenceState(SequenceState.PRELOAD_SHOT);
        stopTransferAction.run();
    }

    @Override
    public void start() {
        segmentStarted = false;
        segmentTimer.reset();
        shotTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        configureSequenceIfNeeded();
        runCurrentSequence();

        PoseMemory.lastPose = follower.getPose();

        telemetry.addData("Sequence",       sequenceState);
        telemetry.addData("Path Busy",      follower.isBusy());
        telemetry.addData("Shot Triggered", shotTriggered);
        telemetry.addData("Shot Finished",  shotFinished);
        telemetry.addData("Pose X",         follower.getPose().getX());
        telemetry.addData("Pose Y",         follower.getPose().getY());
        telemetry.addData("Pose Heading",   follower.getPose().getHeading());
        telemetry.addData("Segment Time",   segmentTimer.seconds());
        telemetry.addData("Shot Time",      shotTimer.seconds());
        telemetry.update();
    }

    private void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
                )
                .build();

        driveShootToHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanZone))
                .setTangentHeadingInterpolation()
                .build();

        driveHumanToBack = follower.pathBuilder()
                .addPath(new BezierLine(humanZone, backPose))
                .setLinearHeadingInterpolation(humanZone.getHeading(), backPose.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();

        driveBackToHuman = follower.pathBuilder()
                .addPath(new BezierLine(backPose, humanZone))
                .setLinearHeadingInterpolation(backPose.getHeading(), humanZone.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();
        driveHumanToShoot = follower.pathBuilder()
                .addPath(new BezierLine(humanZone, shootPose))
                .setLinearHeadingInterpolation(humanZone.getHeading(), shootPose.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();


        driveShoottoSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlSpike3, spike3pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike3pose.getHeading())
                .build();

        driveSpike3ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(spike3pose, controlSpike3, shootPose))
//                .setLinearHeadingInterpolation(spike2Pose.getHeading(), shootPose.getHeading())
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
                )
                .build();

        driveShootToOverflow = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
                )
                .build();

        driveShoottoOverflow2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflowtoShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
                )
                .build();

        driveShootToOverflow3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflow3ToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
//                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
                )
                .build();
    }

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
        transferCurrentlyOpen = false;
        shotPulseCount = 0;
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
                nextSequenceState = SequenceState.TOSpike3;
                break;
            case ToHumanZone:
                currentPath       = driveShootToHuman;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.HumanToBack;
                break;
            case HumanToBack:
                currentPath      = driveHumanToBack;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.BACK_TO_HUMAN;
                break;
            case BACK_TO_HUMAN:
                currentPath = driveBackToHuman;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.HUMAN_TO_SHOOT;
                break;
            case HUMAN_TO_SHOOT:
                currentPath = driveHumanToShoot;
                intakeDuringPath = true;
                shootDuringPath = true;
                nextSequenceState = SequenceState.ToOverflow;
                break;
            case TOSpike3:
                currentPath       = driveShoottoSpike3;
                intakeDuringPath  = true;
                nextSequenceState = SequenceState.RETURN_SHOT_2;
                break;
            case RETURN_SHOT_2:
                currentPath       = driveSpike3ToShoot;
                shootDuringPath   = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.ToHumanZone;
                break;
            case ToOverflow:
                currentPath       = driveShootToOverflow;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.RETURN_SHOT_3;
                break;
            case RETURN_SHOT_3:
                currentPath       = driveOverflowToShoot;
                shootDuringPath   = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_2;
                break;
            case TO_GATE_2:
                currentPath       = driveShoottoOverflow2;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.RETURN_SHOT_4;
                break;
            case RETURN_SHOT_4:
                currentPath       = driveOverflowtoShoot2;
                shootDuringPath   = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_3;
                break;
            case TO_GATE_3:
                currentPath       = driveShootToOverflow3;
                intakeDuringPath  = true;
                waitAtEndOfPath   = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case RETURN_SHOT_5:
                currentPath       = driveOverflow3ToShoot3;
                shootDuringPath   = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case FINISHED:
                stopTransferAction.run();
                shootAction.stop();
                intakeAction.stop();
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

        // Stop transfer at start of every segment
        stopTransferAction.run();

        // Start or stop intake immediately based on segment type
        if (intakeDuringPath) {
            intakeAction.startIntake();
        } else {
            intakeAction.stop();
        }
    }

    private void runCurrentSequence() {
        if (sequenceState == SequenceState.FINISHED) return;

        Pose currentPose = follower.getPose();


        if (shootDuringPath) {
//            int activeRPM = useShot2Preset() ? targetRPM2 : targetRPM1;
//            double activeHoodDeg = useShot2Preset() ? hoodAngle2Deg : hoodAngle1Deg;
            int activeRPM = targetRPM1;
            double activeHoodDeg = hoodAngle1Deg;

            flyWheelSubsystem.spinUp(activeRPM);
            flyWheelSubsystem.setHoodAngle(activeHoodDeg);

            boolean flywheelReady = flyWheelSubsystem.isAtSpeed(activeRPM);

            boolean pathDone           = !follower.isBusy();


//            if (!shotTriggered && pathDone) {
//                transferAction.start();
//                shotTriggered      = true;
//                pathEndWaitStarted = false;
//                shotTimer.reset();
//
//            }
//
//            // Stop transfer after fireDurationSeconds
//            if (shotTriggered && !shotFinished && shotTimer.seconds() >= fireDurationSeconds) {
//                stopTransferAction.run();
//                shotFinished = true;
//            }
//
//            // Start timeout once path finishes and shot still hasn't triggered
//            if (!shotTriggered && pathDone && !pathEndWaitStarted) {
//                pathEndWaitStarted = true;
//                pathEndTimer.reset();
//            }
//
//            // Timeout: skip shot and move on
//            if (!shotTriggered && pathDone && pathEndWaitStarted
//                    && pathEndTimer.seconds() >= maxWaitForShotSeconds) {
//                stopTransferAction.run();
//                shotFinished = true;
//            }
//        } else {
//            shootAction.idle();

            if (pathDone && !shotTriggered && !shotFinished) {
                // Start the wait timer once path finishes
                if (!pathEndWaitStarted) {
                    pathEndWaitStarted = true;
                    pathEndTimer.reset();
                }

                // Fire as soon as flywheel is ready
                if (flywheelReady) {
                    transferSubsystem.Open();
                    transferAction.start();
                    shotTriggered = true;
                    transferCurrentlyOpen = true;
                    shotPulseCount = 1;
                    shotTimer.reset();
                }
                // Timeout — flywheel never got ready, skip shot
                else if (pathEndTimer.seconds() >= maxWaitForShotSeconds) {
                    stopTransferAction.run();
                    shotFinished = true;
                }
            }

            if (shotTriggered && !shotFinished) {
                if (transferCurrentlyOpen && shotTimer.seconds() >= transferOpenSeconds) {
                    transferSubsystem.Closed();
                    transferCurrentlyOpen = false;
                    shotTimer.reset();
                } else if (!transferCurrentlyOpen
                        && shotPulseCount < shotPulseCountTarget
                        && shotTimer.seconds() >= transferClosedSeconds) {
                    transferSubsystem.Open();
                    transferCurrentlyOpen = true;
                    shotPulseCount++;
                    shotTimer.reset();
                } else if (!transferCurrentlyOpen
                        && shotPulseCount >= shotPulseCountTarget
                        && shotTimer.seconds() >= transferClosedSeconds) {
                    intakeAction.stop();
                    shotFinished = true;
                }
            }

        } else {
            shootAction.idle();
        }

        // Keep intake running during intake segments until shot triggers
        if (intakeDuringPath) {
            intakeAction.startIntake();
        }

        // Hold timer at gate end
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

        boolean holdComplete = !waitAtEndOfPath || (holdStarted && holdTimer.seconds() >= gateWaitSeconds);

        if (!follower.isBusy() && shotFinished && holdComplete) {
            setSequenceState(nextSequenceState);
        }
    }

}
