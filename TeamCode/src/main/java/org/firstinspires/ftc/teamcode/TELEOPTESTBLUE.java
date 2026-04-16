package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.COMMAND.DriveCommand;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.*;




@TeleOp(name = "TeleOp Test BLUE")
public class TELEOPTESTBLUE extends CommandOpMode {

    // -------------------- Subsystems --------------------
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;

    // -------------------- Pedro --------------------
    private Follower follower;

    // -------------------- Gamepad --------------------
    private GamepadEx gamepadEx;

    private GamepadEx gamepadEx2;

    // -------------------- Shooter Calculator --------------------
    // Swap GoalTarget.BLUE / RED to match your alliance
    private final SHOOTERCALCBLUE shooterCalc =
            new SHOOTERCALCBLUE();
    private SHOOTERCALCBLUE.ShotSolution shotSolution = null;

    // -------------------- Tuning increments --------------------
    private final double rpmIncrement   = 50;
    private final double angleIncrement = 2;

    // -------------------- Dpad edge-detection --------------------
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

    // -------------------- State flags --------------------
    private boolean gateOpen         = false;
    private boolean flywheelEnabled  = true;
    private boolean headingLocked     = false;
    private boolean manualRPMOverride = false;

    // -------------------- Heading lock timer --------------------
    private long headingLockStartTime          = 0;
    private static final long HEADING_LOCK_DURATION_MS = 1500;

    // ====================================================================
    @Override
    public void initialize() {

        // --- Pedro setup ---
        // Replace FConstants / LConstants with your actual Pedro constants class names
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36.4, 132.2, Math.toRadians(90)));
        // --- Subsystems ---
        driveSubsystem    = new DriveSubsystem(hardwareMap);
        intakeSubsystem   = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        transferSubsystem.Closed();
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // --- Gamepad ---
        gamepadEx = new GamepadEx(gamepad1);

        // --- Telemetry (also mirrors to FTC Dashboard) ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ======================== DRIVE ========================
        new DriveCommand(
                driveSubsystem,
                () -> (double) -gamepad1.left_stick_y,
                () -> (double) -gamepad1.right_stick_x,
                () -> (double) -gamepad1.left_stick_x
        ).schedule();

        // ======================== FLYWHEEL ========================
        // Priority:
        //   flywheelEnabled=false          → stop
        //   headingLocked + !manualOverride → calculator RPM + hood
        //   headingLocked + manualOverride  → manual RPM + manual hood (heading still locked)
        //   no lock                         → manual RPM + manual hood
//        flyWheelSubsystem.setDefaultCommand(
//                new RunCommand(() -> {
//                    if (!flywheelEnabled) {
//                        flyWheelSubsystem.stop();
//                        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
//                        return;
//                    }
//                    if (headingLocked && shotSolution != null && !manualRPMOverride) {
//                        // Full auto mode
//                        flyWheelSubsystem.spinUp(shotSolution.rpm);
//                        flyWheelSubsystem.setHoodAngle(shotSolution.hoodAngleDeg);
//                    } else {
//                        // Manual mode (covers: no lock, or manual override with lock)
//                        flyWheelSubsystem.spinUp(FlyWheelConstants.targetRPM);
//                        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
//                    }
//                }, flyWheelSubsystem)
//        );

        flyWheelSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (!flywheelEnabled) {
                        flyWheelSubsystem.stop();
                        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
                        return;
                    }
                    if (headingLocked) {
                        // Full auto mode
                        flyWheelSubsystem.spinUp(FlyWheelConstants.targetRPM);
                        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
                    } else {
                        // Manual mode (covers: no lock, or manual override with lock)
                        flyWheelSubsystem.spinUp(FlyWheelConstants.targetRPM);
                        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
                    }
                }, flyWheelSubsystem)
        );

        // X → flywheel on
        new GamepadButton(gamepadEx, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = true));

        // B → flywheel off
        new GamepadButton(gamepadEx, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = false));

        // Y → toggle manual RPM override
        new GamepadButton(gamepadEx, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    flyWheelSubsystem.spinUp(3200);
                    flyWheelSubsystem.setHoodAngle(42);
                }));

        // ======================== A: AIM + TRANSFER ========================
        // Locks heading toward goal, opens transfer gate for 2 seconds, then releases both.
        // If manualRPMOverride is ON, heading still locks but RPM/hood stay manual.
        new GamepadButton(gamepadEx, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    gateOpen = true;
                    transferSubsystem.Open();
                })
                .whenReleased(() -> {
                    gateOpen = false;
                    transferSubsystem.Closed();
                });

        new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    headingLocked = true;
                    headingLockStartTime =System.currentTimeMillis();
                }));

        // ======================== INTAKE ========================
        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (gateOpen) {
                        intakeSubsystem.transfer();
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeSubsystem.runAll();
                    } else if (gamepad1.left_trigger > 0.1) {
                        intakeSubsystem.reverseAll();
                    } else {
                        intakeSubsystem.off();
                    }
                }, intakeSubsystem));

        // ======================== MAIN LOOP COMMAND ========================
        new RunCommand(() -> {

            // Update Pedro localizer
            follower.update();

            // --- Heading lock timeout ---
            if (headingLocked) {
                long elapsed = System.currentTimeMillis() - headingLockStartTime;
                if (elapsed >= HEADING_LOCK_DURATION_MS) {
                    headingLocked = false;
                    gateOpen      = false;
                    transferSubsystem.Closed();
                }
            }

            // --- Dpad tuning: RPM and hood angle (always active) ---
            if (gamepad1.dpad_up    && !prevDpadUp)    FlyWheelConstants.targetRPM += rpmIncrement;
            if (gamepad1.dpad_down  && !prevDpadDown)  FlyWheelConstants.targetRPM -= rpmIncrement;
            if (gamepad1.dpad_right && !prevDpadRight) FlyWheelConstants.hoodAngle += angleIncrement;
            if (gamepad1.dpad_left  && !prevDpadLeft)  FlyWheelConstants.hoodAngle -= angleIncrement;

            // Clamp values
            FlyWheelConstants.targetRPM = Math.max(0, Math.min(6000, FlyWheelConstants.targetRPM));
            FlyWheelConstants.hoodAngle = Math.max(
                    FlyWheelConstants.hoodMinAngleDeg(),
                    Math.min(FlyWheelConstants.hoodMaxAngleDeg(), FlyWheelConstants.hoodAngle)
            );

            // Edge detection update
            prevDpadUp    = gamepad1.dpad_up;
            prevDpadDown  = gamepad1.dpad_down;
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft  = gamepad1.dpad_left;

        }).schedule();
    }

    // ====================================================================
    @Override
    public void run() {
        super.run();

        // --- Get pose ---
        Pose   robotPose = follower.getPose();
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        double rh = robotPose.getHeading(); // radians

        // --- Get velocity (graceful fallback to 0,0 if not available) ---
        double vx = 0, vy = 0;
        try {
            vx = follower.getVelocity().getXComponent();
            vy = follower.getVelocity().getYComponent();
        } catch (Exception ignored) {}

        // --- Recompute shot solution every loop ---
        shotSolution = shooterCalc.calculateShotSolution(rx, ry, rh, vx, vy);

//        // --- Heading correction when locked ---
//        if (headingLocked && shotSolution != null) {
//            driveSubsystem.setHeadingOverride(
//                    shooterCalc.headingTurnPower(shotSolution.headingErrorRad)
//            );
//        } else {
//            driveSubsystem.clearHeadingOverride();
//        }
//
//        double headingToGoal = Math.atan((144-ry)/rx);

        // ======================== TELEMETRY ========================

        telemetry.addData("X (in)",         rx);
        telemetry.addData("Y (in)",         ry);
        telemetry.addData("Heading (deg)",  Math.toDegrees(rh));
//        telemetry.addData("Distance (in)",  shotSolution != null ? shotSolution.distanceInches : 0);

        telemetry.addData("Target RPM",     FlyWheelConstants.targetRPM);
        telemetry.addData("Target Hood",    FlyWheelConstants.hoodAngle);
        telemetry.addData("Top RPM",        flyWheelSubsystem.getCurrentRPMTOP());
        telemetry.addData("Bottom RPM",     flyWheelSubsystem.getCurrentRPMBottom());
        telemetry.addData("RPM Error",      FlyWheelConstants.targetRPM - flyWheelSubsystem.getCurrentRPMTOP());
        telemetry.addData("At Speed",       flyWheelSubsystem.isAtSpeed(FlyWheelConstants.targetRPM));
        telemetry.addData("Voltage",        flyWheelSubsystem.voltageSensor.getVoltage());

        telemetry.update();
    }
}