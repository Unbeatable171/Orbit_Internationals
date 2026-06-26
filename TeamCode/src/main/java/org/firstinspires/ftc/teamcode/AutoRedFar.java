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
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.Turrettt;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

@Config
@Autonomous(name = "Auto Red Far")
public class AutoRedFar extends OpMode {

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
        HUMAN_TO_SHOOT,
        RETURN_SHOT_6,
        TO_GATE_4
    }

    public static double fireDurationSeconds = 0.8;
    public static double maxWaitForShotSeconds = 2.5;
    public static double gateWaitSeconds = 0.8;
    public static double rpmTolerance = 125.0;

    public static double redGoalXInches = 126.5;
    public static double redGoalYInches = 136;

    public static int targetRPM1 = 3000;
    public static double hoodAngle1Deg = 45;

    public static int targetRPM2 = 2450;
    public static double hoodAngle2Deg = 60;
    public static double turretShot1Pos = 0.475;
    public static double turretShot2Pos = 0.475;

    private final Pose startPose = new Pose(85.5, 8.9, Math.toRadians(90));
    private final Pose shootPose = new Pose(85.38413685847589, 13.20373250388803, Math.toRadians(65.5));

    private final Pose humanZone = new Pose(136.5, 8, Math.toRadians(-40));
    private final Pose backPose = new Pose(110, 8, Math.toRadians(0));

    private final Pose spike3pose = new Pose(116.5, 32, Math.toRadians(0));
    private final Pose controlSpike3 = new Pose(95.91642498392333, 35);

    private final Pose overflowPick = new Pose(128.61, 38.12498924965115, Math.toRadians(60));
    private final Pose overflowControlPose = new Pose(122.6064359525804, 1.0286688083403455);

    private Follower follower;
    private FlyWheelSubsystem flyWheelSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private Turrettt turret;

    private ShootAction shootAction;
    private TransferAction transferAction;
    private StopTransferAction stopTransferAction;
    private IntakeAction intakeAction;

    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime pathEndTimer = new ElapsedTime();

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
    private PathChain driveOverflow4ToShoot4;
    private PathChain driveShootToOverflow4;

    @Override
    public void init() {
        RobotConstants.chosenAlliance = "RED";

        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        turret = new Turrettt(hardwareMap);

        shootAction = new ShootAction(flyWheelSubsystem);
        transferAction = new TransferAction(transferSubsystem, intakeSubsystem);
        stopTransferAction = new StopTransferAction(transferSubsystem, intakeSubsystem);
        intakeAction = new IntakeAction(intakeSubsystem);

        follower = Pedropathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        setSequenceState(SequenceState.PRELOAD_SHOT);
        stopTransferAction.run();
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
        follower.update();
        updateTurret();

        configureSequenceIfNeeded();
        runCurrentSequence();

        PoseMemory.lastPose = follower.getPose();

        telemetry.addData("Sequence", sequenceState);
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Shot Triggered", shotTriggered);
        telemetry.addData("Shot Finished", shotFinished);
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Pose Heading", follower.getPose().getHeading());
        telemetry.addData("Segment Time", segmentTimer.seconds());
        telemetry.addData("Shot Time", shotTimer.seconds());
        telemetry.addData("Turret Pos", turret.getServoPosition());
        telemetry.addData("Turret Target", turret.getFinalTargetPosition());
        telemetry.addData("Turret Routing", turret.isRoutingThroughMiddle());
        telemetry.addData("Turret OnTarget", isTurretOnTarget());
        telemetry.update();
    }

    private void updateTurret() {
        if (isTurretActiveState()) {
            double targetServoPos = useShot2Preset()
                    ? turretShot2Pos
                    : turretShot1Pos;

            turret.setPosition(targetServoPos);
        } else {
            turret.commandPosition(turretShot1Pos);
        }
    }

    private boolean isTurretActiveState() {
        switch (sequenceState) {
            case PRELOAD_SHOT:
            case HUMAN_TO_SHOOT:
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

    private void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();

        driveShootToHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanZone))
                .setTangentHeadingInterpolation()
                .build();

        driveHumanToBack = follower.pathBuilder()
                .addPath(new BezierLine(humanZone, backPose))
                .setLinearHeadingInterpolation(humanZone.getHeading(), backPose.getHeading())
                .build();

        driveBackToHuman = follower.pathBuilder()
                .addPath(new BezierLine(backPose, humanZone))
                .setLinearHeadingInterpolation(backPose.getHeading(), humanZone.getHeading())
                .build();

        driveHumanToShoot = follower.pathBuilder()
                .addPath(new BezierLine(humanZone, shootPose))
                .setLinearHeadingInterpolation(humanZone.getHeading(), shootPose.getHeading())
                .build();

        driveShoottoSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlSpike3, spike3pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike3pose.getHeading())
                .build();

        driveSpike3ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(spike3pose, controlSpike3, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();

        driveShootToOverflow = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();

        driveShoottoOverflow2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflowtoShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();

        driveShootToOverflow3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflow3ToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();

        driveShootToOverflow4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, overflowControlPose, overflowPick))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPick.getHeading())
                .build();

        driveOverflow4ToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPick, shootPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(redGoalXInches, redGoalYInches)
                )
                .build();
    }

    private void setSequenceState(SequenceState newState) {
        sequenceState = newState;
        segmentStarted = false;
        segmentTimer.reset();
        shotTimer.reset();
        shotTriggered = false;
        shotFinished = false;
        waitAtEndOfPath = false;
        holdStarted = false;
        pathEndWaitStarted = false;
    }

    private void configureSequenceIfNeeded() {
        if (segmentStarted) return;

        shootDuringPath = false;
        intakeDuringPath = false;
        waitAtEndOfPath = false;
        currentPath = null;
        nextSequenceState = SequenceState.FINISHED;

        switch (sequenceState) {
            case PRELOAD_SHOT:
                currentPath = driveStartToShoot;
                shootDuringPath = true;
                nextSequenceState = SequenceState.TOSpike3;
                break;
            case ToHumanZone:
                currentPath = driveShootToHuman;
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
                currentPath = driveShoottoSpike3;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.RETURN_SHOT_2;
                break;
            case RETURN_SHOT_2:
                currentPath = driveSpike3ToShoot;
                shootDuringPath = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.ToHumanZone;
                break;
            case ToOverflow:
                currentPath = driveShootToOverflow;
                intakeDuringPath = true;
                waitAtEndOfPath = true;
                nextSequenceState = SequenceState.RETURN_SHOT_3;
                break;
            case RETURN_SHOT_3:
                currentPath = driveOverflowToShoot;
                shootDuringPath = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_2;
                break;
            case TO_GATE_2:
                currentPath = driveShoottoOverflow2;
                intakeDuringPath = true;
                waitAtEndOfPath = true;
                nextSequenceState = SequenceState.RETURN_SHOT_4;
                break;
            case RETURN_SHOT_4:
                currentPath = driveOverflowtoShoot2;
                shootDuringPath = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_3;
                break;
            case TO_GATE_3:
                currentPath = driveShootToOverflow3;
                intakeDuringPath = true;
                waitAtEndOfPath = true;
                nextSequenceState = SequenceState.RETURN_SHOT_5;
                break;
            case RETURN_SHOT_5:
                currentPath = driveOverflow3ToShoot3;
                shootDuringPath = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.TO_GATE_4;
                break;
            case TO_GATE_4:
                currentPath = driveShootToOverflow4;
                shootDuringPath = true;
                intakeDuringPath = true;
                nextSequenceState = SequenceState.FINISHED;
                break;
            case FINISHED:
                stopTransferAction.run();
                shootAction.stop();
                intakeAction.stop();
                turret.commandPosition(Turrettt.safeMiddle);
                shotFinished = true;
                segmentStarted = true;
                return;
            default:
                setSequenceState(SequenceState.FINISHED);
                return;
        }

        follower.followPath(currentPath, true);
        segmentStarted = true;
        shotTriggered = !shootDuringPath;
        shotFinished = !shootDuringPath;
        holdStarted = false;
        pathEndWaitStarted = false;

        segmentTimer.reset();
        shotTimer.reset();
        holdTimer.reset();
        pathEndTimer.reset();

        stopTransferAction.run();

        if (intakeDuringPath) {
            intakeAction.startIntake();
        } else {
            intakeAction.stop();
        }
    }

    private void runCurrentSequence() {
        if (sequenceState == SequenceState.FINISHED) return;

        if (shootDuringPath) {
            int activeRPM = targetRPM1;
            double activeHoodDeg = hoodAngle1Deg;

            flyWheelSubsystem.spinUp(activeRPM);
            flyWheelSubsystem.setHoodAngle(activeHoodDeg);

            boolean flywheelReady = isAtSpeed(activeRPM);
            boolean pathDone = !follower.isBusy();

            if (pathDone && !shotTriggered && !shotFinished) {
                if (!pathEndWaitStarted) {
                    pathEndWaitStarted = true;
                    pathEndTimer.reset();
                }

                if (flywheelReady && isTurretOnTarget()) {
                    transferAction.start();
                    shotTriggered = true;
                    shotTimer.reset();
                } else if (pathEndTimer.seconds() >= maxWaitForShotSeconds) {
                    stopTransferAction.run();
                    shotFinished = true;
                }
            }

            if (shotTriggered && !shotFinished && shotTimer.seconds() >= fireDurationSeconds) {
                stopTransferAction.run();
                intakeAction.stop();
                shotFinished = true;
            }
        } else {
            shootAction.idle();
        }

        if (intakeDuringPath) {
            intakeAction.startIntake();
        }

        if (!follower.isBusy()) {
            if (waitAtEndOfPath && !holdStarted) {
                holdStarted = true;
                holdTimer.reset();
            }
        } else {
            holdStarted = false;
            pathEndWaitStarted = false;
            holdTimer.reset();
            pathEndTimer.reset();
        }

        boolean holdComplete = !waitAtEndOfPath || (holdStarted && holdTimer.seconds() >= gateWaitSeconds);

        if (!follower.isBusy() && shotFinished && holdComplete) {
            setSequenceState(nextSequenceState);
        }
    }

    private boolean isAtSpeed(double targetRPM) {
        return Math.abs(flyWheelSubsystem.getCurrentRPMLeft() - targetRPM) <= rpmTolerance
                && Math.abs(flyWheelSubsystem.getCurrentRPMRight() - targetRPM) <= rpmTolerance;
    }

    private boolean useShot2Preset() {
        switch (sequenceState) {
            case HUMAN_TO_SHOOT:
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

    private boolean isTurretOnTarget() {
        return !turret.isRoutingThroughMiddle()
                && Math.abs(turret.getServoPosition() - turret.getFinalTargetPosition()) < 0.01;
    }

    @Override
    public void stop() {
        PoseMemory.lastPose = follower.getPose();
    }
}
