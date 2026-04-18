package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.COMMAND.DriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.*;




@TeleOp(name = "TeleOp Test 3")
public class TELEOPTEST3 extends CommandOpMode {

    // -------------------- Subsystems --------------------
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;
    private double currentTargetRPM = FlyWheelConstants.targetRPM;
    private double currentHoodAngle = FlyWheelConstants.hoodAngle;
    private double rotate = 0;
    private double targetHeadingRad = 0;
    private double headingIntegral = 0;
    private double previousHeadingError = 0;
    private long lastHeadingPidTimeNanos = 0;

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
    private boolean pedrodrive = false;

    // ====================================================================
    @Override
    public void initialize() {

        // --- Pedro setup ---
        // Replace FConstants / LConstants with your actual Pedro constants class names
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(37.34831836370614, 136.13465551339004, Math.toRadians(90)));
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
                () ->  pedrodrive ? 0.0 : (double) -gamepad1.left_stick_y,
                () -> pedrodrive ? 0.0 : rotate,
                () -> pedrodrive? 0.0: (double) - gamepad1.left_stick_x
        ).schedule();

        new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    headingLocked = true;
                    resetHeadingController();
                })
                .whenReleased(() -> {
                    headingLocked = false;
                    rotate = -gamepad1.right_stick_x ;
                    resetHeadingController();
                });




        flyWheelSubsystem.setDefaultCommand(
                    new RunCommand(() -> {
                        if (!flywheelEnabled) {
                            flyWheelSubsystem.stop();
                            flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
                        } else if (shotSolution != null) {
                            flyWheelSubsystem.spinUp(shotSolution.rpm);
                            flyWheelSubsystem.setHoodAngle(shotSolution.hoodAngleDeg);
                        } else {
                            flyWheelSubsystem.spinUp(currentTargetRPM);
                            flyWheelSubsystem.setHoodAngle(currentHoodAngle);
                        }
                    }, flyWheelSubsystem));

        // X → flywheel on
        new GamepadButton(gamepadEx2, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = true));

        // B → flywheel off
        new GamepadButton(gamepadEx2, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = false));

        // Y → toggle manual RPM override
        new GamepadButton(gamepadEx2, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    currentTargetRPM = 3200;
                    currentHoodAngle = 47;
                }));

        // ======================== A: AIM + TRANSFER ========================
        // Locks heading toward goal, opens transfer gate for 2 seconds, then releases both.
        // If manualRPMOverride is ON, heading still locks but RPM/hood stay manual.
        new GamepadButton(gamepadEx2, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    gateOpen = true;
                    transferSubsystem.Open();
                })
                .whenReleased(() -> {
                    gateOpen = false;
                    transferSubsystem.Closed();
                });

      /*  new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    currentTargetRPM = 2400;
                    currentHoodAngle = 65;
                }));

       */
// RIGHT_BUMPER → mid shot
        new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    currentTargetRPM = 2300;
                    currentHoodAngle = 65;
                }));


        new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    pedrodrive = true;

                    Pose currentPose = follower.getPose();
                    Pose gatePose = new Pose(129.2,58.096423017107305,Math.toRadians(25));

                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(new BezierLine
                                            (currentPose, gatePose))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(),gatePose.getHeading())
                                    .build(),
                            true
                    );
                }))
                .whenReleased(new InstantCommand(()-> {
                    pedrodrive = false;
                    follower.breakFollowing();
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

        // While left bumper is held, keep refreshing the target heading from the current pose.
        if (headingLocked) {
            targetHeadingRad = shotSolution.targetHeadingRad;
            rotate = calculateHeadingPidOutput(rh, targetHeadingRad);
            driveSubsystem.setHeadingOverride(rotate);
        } else {
            rotate = 0;
            driveSubsystem.clearHeadingOverride();
        }

        // ======================== TELEMETRY ========================

        telemetry.addData("X (in)",         rx);
        telemetry.addData("Y (in)",         ry);
        telemetry.addData("Heading (deg)",  Math.toDegrees(rh));
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeadingRad));
        telemetry.addData("Heading Locked", headingLocked);
        telemetry.addData("Rotate Command", rotate);
//        telemetry.addData("Distance (in)",  shotSolution != null ? shotSolution.distanceInches : 0);

        telemetry.addData("Target RPM",     FlyWheelConstants.targetRPM);
        telemetry.addData("Target Hood",    FlyWheelConstants.hoodAngle);
        telemetry.addData("Top RPM",        flyWheelSubsystem.getCurrentRPMTOP());
        telemetry.addData("Bottom RPM",     flyWheelSubsystem.getCurrentRPMBottom());
        telemetry.addData("RPM Error",      FlyWheelConstants.targetRPM - flyWheelSubsystem.getCurrentRPMTOP());
        telemetry.addData("At Speed",       flyWheelSubsystem.isAtSpeed(FlyWheelConstants.targetRPM));
        telemetry.addData("Voltage",        flyWheelSubsystem.voltageSensor.getVoltage());
        telemetry.addData("Current Target RPM", currentTargetRPM);
        telemetry.addData("Current Hood Angle", currentHoodAngle);

        telemetry.update();
    }

    private void resetHeadingController() {
        headingIntegral = 0;
        previousHeadingError = 0;
        lastHeadingPidTimeNanos = System.nanoTime();
    }

    private double calculateHeadingPidOutput(double currentHeadingRad, double desiredHeadingRad) {
        double error = normalizeAngle(desiredHeadingRad - currentHeadingRad);

        long now = System.nanoTime();
        double dt = lastHeadingPidTimeNanos == 0 ? 0.02 : (now - lastHeadingPidTimeNanos) / 1e9;
        lastHeadingPidTimeNanos = now;
        if (dt <= 0) {
            dt = 0.02;
        }

        headingIntegral += error * dt;
        headingIntegral = Math.max(-1.0, Math.min(1.0, headingIntegral));

        double derivative = (error - previousHeadingError) / dt;
        previousHeadingError = error;

        boolean useSecondary = Math.abs(error) <= Constants.TELEOP_SECONDARY_HEADING_ERROR_RAD;
        double p = useSecondary ? Constants.SECONDARY_HEADING_P : Constants.PRIMARY_HEADING_P;
        double i = useSecondary ? Constants.SECONDARY_HEADING_I : Constants.PRIMARY_HEADING_I;
        double d = useSecondary ? Constants.SECONDARY_HEADING_D : Constants.PRIMARY_HEADING_D;
        double f = useSecondary ? Constants.SECONDARY_HEADING_F : Constants.PRIMARY_HEADING_F;

        double output = (p * error) + (i * headingIntegral) + (d * derivative);
        if (Math.abs(error) > 1e-3) {
            output += Math.copySign(f, error);
        }

        return Math.max(-1.0, Math.min(1.0, output));
    }
}
