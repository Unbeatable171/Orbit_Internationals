import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CONSTANTS;
import org.firstinspires.ftc.teamcode.Command.DriveCommand;
import org.firstinspires.ftc.teamcode.PoseMemory;
import org.firstinspires.ftc.teamcode.ShooterCalculatorRed;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

@TeleOp(name = "TeleOp Red Heading")
public class TeleOpRedHeading extends CommandOpMode {

    private static final double rpm1 = 2500;
    private static final double hood1 = CONSTANTS.hoodAngleToServoPosition(50);
    private static final double rpm2 = 2700;
    private static final double hood2 = CONSTANTS.hoodAngleToServoPosition(55);
    private static final double rpm3 = 3300;
    private static final double hood3 = CONSTANTS.hoodAngleToServoPosition(55);

    private static final double HOOD_INCREMENT = 0.02;
    private static final double HOOD_MIN       = 0.56;
    private static final double HOOD_MAX       = 1.0;

    // ── Heading lock ──────────────────────────────────────────────────────────
    public static double HEADING_KP            = 0.8;
    public static double HEADING_TOLERANCE_RAD = 0.05;

    private boolean headingLockEnabled = false;

    private static boolean prevdpadleft = false;
    private static boolean prevdpaddown = false;

    // ── Poses ─────────────────────────────────────────────────────────────────
    private Pose startPose       = new Pose(109.25505443234837, 131.45489891135304, Math.toRadians(90));
    private Pose relocalizePose1 = new Pose(126.63996889580092, 77.09953343701397, Math.toRadians(0));
    private Pose relocalizePose2 = new Pose(9.6, 7.9, Math.toRadians(180));

    private Follower follower;

    private ShooterCalculatorRed shooterCalc = new ShooterCalculatorRed();
    private ShooterCalculatorRed.ShotSolution shotSolution = null;

    private DriveSubsystem    driveSubsystem;
    private IntakeSubsystem   intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;

    private Servo hoodServo;

    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    private boolean transferOpen    = false;
    private boolean flywheelEnabled = true;

    private double manualHoodServoPosition = hood2;

    // =========================================================================
    @Override
    public void initialize() {
        RobotConstants.chosenAlliance = "RED";

        follower = Pedropathing.Constants.createFollower(hardwareMap);
        if (PoseMemory.lastPose != null) {
            startPose = PoseMemory.lastPose;
        }
        follower.setStartingPose(startPose);
        Localization.init(follower);
        shooterCalc = new ShooterCalculatorRed();

        driveSubsystem    = new DriveSubsystem(hardwareMap);
        intakeSubsystem   = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        transferSubsystem.Closed();
        shotSolution = shooterCalc.calculateShotSolution(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        );

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // ── Drive command — rotate supplier switches between stick and heading lock
        new DriveCommand(
                driveSubsystem,
                () -> (double) -gamepad1.left_stick_y,
                () -> (double)  gamepad1.left_stick_x,
                () -> computeRotate()
        ).schedule();

        // ── Intake default command ────────────────────────────────────────────
        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (transferOpen) {
                        intakeSubsystem.transfer();
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeSubsystem.runAll();
                    } else if (gamepad1.left_trigger > 0.1) {
                        intakeSubsystem.reverseAll();
                    } else {
                        intakeSubsystem.off();
                    }
                }, intakeSubsystem)
        );

        // ── Flywheel default command ──────────────────────────────────────────
        flyWheelSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (flywheelEnabled) {
                        flyWheelSubsystem.spinUp(shotSolution.rpm);
                    } else {
                        flyWheelSubsystem.stop();
                    }
                    flyWheelSubsystem.setHoodAngle(shotSolution.hoodAngleDeg);
                }, flyWheelSubsystem)
        );

        // ── Shooter preset buttons ────────────────────────────────────────────
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm1, hood1);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm2, hood2);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm3, hood3);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = false;
                    transferOpen    = false;
                    transferSubsystem.Closed();
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = true;
                    transferOpen    = true;
                    transferSubsystem.Open();
                }))
                .whenReleased(new InstantCommand(() -> {
                    transferOpen = false;
                    transferSubsystem.Closed();
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    manualHoodServoPosition = Math.min(manualHoodServoPosition + HOOD_INCREMENT, HOOD_MAX);
                    hoodServo.setPosition(manualHoodServoPosition);
                    CONSTANTS.hoodAngle = CONSTANTS.servoPositionToHoodAngle(manualHoodServoPosition);
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    manualHoodServoPosition = Math.max(manualHoodServoPosition - HOOD_INCREMENT, HOOD_MIN);
                    hoodServo.setPosition(manualHoodServoPosition);
                    CONSTANTS.hoodAngle = CONSTANTS.servoPositionToHoodAngle(manualHoodServoPosition);
                }));

        // ── Heading lock toggle — gamepad1 LEFT_BUMPER ────────────────────────
        new GamepadButton(gamepad1Ex, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    headingLockEnabled = !headingLockEnabled;
                }))
                .whenReleased(new InstantCommand(()->{
                    headingLockEnabled = false;
                }));
    }

    // ── Heading-lock helpers ──────────────────────────────────────────────────

    private double computeRotate() {
        if (!headingLockEnabled) {
            return gamepad1.right_stick_x;
        }

        Pose pose = follower.getPose();
        double targetHeading = goalHeadingRad(pose.getX(), pose.getY());
        double error = angleWrap(targetHeading - pose.getHeading());

        if (Math.abs(error) < HEADING_TOLERANCE_RAD) return 0.0;

        return -Math.max(-1.0, Math.min(1.0, HEADING_KP * error));
    }

    private double goalHeadingRad(double x, double y) {
        double dx = 139- x;
        double dy = 131 - y;
        return Math.atan2(dy, dx);
    }

    private double angleWrap(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ── Relocalization helpers ────────────────────────────────────────────────

    private void relocalizePedro1() {
        follower.setPose(relocalizePose1);
        follower.startTeleopDrive(true);
    }

    private void relocalizePedro2() {
        follower.setPose(relocalizePose2);
        follower.startTeleopDrive(true);
    }

    private void setPreset(double rpm, double hoodPosition) {
        CONSTANTS.targetRPM     = rpm;
        CONSTANTS.hoodAngle     = CONSTANTS.servoPositionToHoodAngle(hoodPosition);
        manualHoodServoPosition = hoodPosition;
        hoodServo.setPosition(hoodPosition);
    }

    // =========================================================================
    @Override
    public void run() {
        Localization.update();
        super.run();

        if (gamepad1.dpad_left && !prevdpadleft) relocalizePedro1();
        if (gamepad1.dpad_down && !prevdpaddown) relocalizePedro2();

        prevdpadleft = gamepad1.dpad_left;
        prevdpaddown = gamepad1.dpad_down;

        Pose robotPose = follower.getPose();
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        double rh = robotPose.getHeading();

        double vx = 0, vy = 0;
        try {
            vx = follower.getVelocity().getXComponent();
            vy = follower.getVelocity().getYComponent();
        } catch (Exception ignored) {}

        shotSolution = shooterCalc.calculateShotSolution(rx, ry, rh, vx, vy);

        double distanceToGoal      = shooterCalc.distanceToGoalInches(rx, ry, rh);
        double currentRpmLeft      = flyWheelSubsystem.getCurrentRPMLeft();
        double currentRpmRight     = flyWheelSubsystem.getCurrentRPMRight();
        double currentHoodAngle    = flyWheelSubsystem.getHoodAngle();
        double currentHoodServoPos = hoodServo.getPosition();

        double targetHeading = goalHeadingRad(rx, ry);
        double headingError  = angleWrap(targetHeading - rh);

        telemetry.addData("X (in)",           rx);
        telemetry.addData("Y (in)",           ry);
        telemetry.addData("Heading (deg)",    Math.toDegrees(rh));
        telemetry.addData("Alliance",         RobotConstants.chosenAlliance);
        telemetry.addData("Distance to Goal", distanceToGoal);

        telemetry.addLine("----------------------------");
        telemetry.addData("Heading Lock",            headingLockEnabled ? "ON" : "OFF");
        telemetry.addData("Target Heading (deg)",    Math.toDegrees(targetHeading));
        telemetry.addData("Heading Error (deg)",     Math.toDegrees(headingError));

        telemetry.addLine("----------------------------");
        telemetry.addData("Target RPM",       CONSTANTS.targetRPM);

        telemetry.addLine("----------------------------");
        telemetry.addData("Left RPM",         currentRpmLeft);
        telemetry.addData("Left RPM Error",   CONSTANTS.targetRPM - currentRpmLeft);

        telemetry.addLine("----------------------------");
        telemetry.addData("Right RPM",        currentRpmRight);
        telemetry.addData("Right RPM Error",  CONSTANTS.targetRPM - currentRpmRight);

        telemetry.addLine("----------------------------");
        telemetry.addData("Hood Angle (deg)",         currentHoodAngle);
        telemetry.addData("Hood Target Angle (deg)",  CONSTANTS.hoodAngle);
        telemetry.addData("Hood Angle Error (deg)",   CONSTANTS.hoodAngle - currentHoodAngle);

        telemetry.addLine("----------------------------");
        telemetry.addData("Hood Servo Position",        currentHoodServoPos);
        telemetry.addData("Hood Servo Target Position", manualHoodServoPosition);
        telemetry.addData("Hood Servo Error",           manualHoodServoPosition - currentHoodServoPos);

        telemetry.update();
    }
}