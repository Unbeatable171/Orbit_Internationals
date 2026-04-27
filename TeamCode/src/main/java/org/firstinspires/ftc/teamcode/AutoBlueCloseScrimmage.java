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
@Autonomous(name = "Auto Blue Close ")
public class AutoBlueCloseScrimmage extends OpMode {

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

    // --- Tunable parameters ---
    public static double fireDurationSeconds   = 0.6;  //earlier value is 1
    public static double maxWaitForShotSeconds = 2.5;
    public static double gateWaitSeconds       = 1.0;

    // Fixed shooter values — tune hoodPosition for your 65 deg target
    public static int targetRPM1 = 2375;
    public static double hoodAngle1Deg = 65;   // degrees, not servo position

    public static int targetRPM2 = 2450;
    public static double hoodAngle2Deg = 62;   // degrees, not servo position

    private SHOOTERCALCBLUE shootercalc = new SHOOTERCALCBLUE();
    private SHOOTERCALCBLUE.ShotSolution shotSolution = null;



    private boolean useShot2Preset() {
        return sequenceState == SequenceState.RETURN_SHOT_1
                || sequenceState == SequenceState.RETURN_SHOT_2
                || sequenceState == SequenceState.RETURN_SHOT_3
                || sequenceState == SequenceState.RETURN_SHOT_4
                || sequenceState == SequenceState.RETURN_SHOT_5;
    }


    private static final double INCHES_TO_METERS = 0.0254;

    private final Pose goalPoseBlue = new Pose(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches);

    // --- Poses ---
    private final Pose startPose  = new Pose(37.34831836370614,  136.13465551339004, Math.toRadians(90));
    private final Pose shootPose  = new Pose(58.6,   83.85628742514972, Math.toRadians(130));
    private final Pose shootPose2 = new Pose(59.5,80,Math.toRadians(132));

    // Spike 1
    private final Pose spike1Pose        = new Pose(36, 82.39819893649711, Math.toRadians(180));
    private final Pose spike1ControlPose = new Pose(48, 74.2);


    // Spike 2
    private final Pose spike2Pose               = new Pose(28.5, 56, Math.toRadians(180));
    private final Pose spike2Control1Pose       = new Pose(70,  50);
    private final Pose spike2ReturnControl1Pose = new Pose(85, 83);

    // Gate
    private final Pose gatePose              = new Pose(21.8, 57.7, Math.toRadians(145));
    private final Pose gateControlPose       = new Pose(41.428710947133744, 64.18349791356334);
    private final Pose gateReturnControlPose = new Pose(25,  33.65863141524106);

    private final Pose leavePose = new Pose(25,56.7, Math.toRadians(145));

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

    private PathChain currentPath;

    // --- Paths ---
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
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootToSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, spike1ControlPose, spike1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Pose.getHeading())
                .build();

        driveSpike1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pose, shootPose2))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), shootPose2.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();

        driveShootToSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, spike2Control1Pose, spike2Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), spike2Pose.getHeading())
                .build();

        driveSpike2ToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(spike2Pose, spike2ReturnControl1Pose, shootPose2))
                .addPath(new BezierCurve(spike2Pose, spike2ReturnControl1Pose, shootPose2))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(), shootPose2.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();

        driveShootToGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate1ToShoot = follower.pathBuilder()
                .setBrakingStart(0.8)
                .addPath(new BezierCurve(gatePose, gateReturnControlPose,shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();

        driveShootToGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();

        driveShootToGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gatePose.getHeading())
                .build();

        driveGate3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                )
                .build();
        driveShootToGate4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2,leavePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),leavePose.getHeading())
                .build();
        driveGate4ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose,shootPose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),gatePose.getHeading())
//                .setHeadingInterpolation(
//                        HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches))
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
                intakeDuringPath = true;
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
                intakeDuringPath = true;
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
                intakeDuringPath = true;
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
                intakeDuringPath = true;
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
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_4;
                break;
            case TO_GATE_4:
                currentPath = driveShootToGate4;
                intakeDuringPath = true;
                waitAtEndOfPath = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case RETURN_SHOT_6:
                currentPath = driveGate4ToShoot;
                shootDuringPath = true;
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
            int activeRPM = useShot2Preset() ? targetRPM2 : targetRPM1;
            double activeHoodDeg = useShot2Preset() ? hoodAngle2Deg : hoodAngle1Deg;

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
                    transferAction.start();
                    shotTriggered = true;
                    shotTimer.reset();
                }
                // Timeout — flywheel never got ready, skip shot
                else if (pathEndTimer.seconds() >= maxWaitForShotSeconds) {
                    stopTransferAction.run();
                    shotFinished = true;
                }
            }

            // Stop transfer after fireDurationSeconds once shot was triggered
            if (shotTriggered && !shotFinished && shotTimer.seconds() >= fireDurationSeconds) {
                stopTransferAction.run();
                shotFinished = true;
            }

        } else {
            shootAction.idle();
        }

        // Keep intake running during intake segments until shot triggers
        if (intakeDuringPath && !shotTriggered) {
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