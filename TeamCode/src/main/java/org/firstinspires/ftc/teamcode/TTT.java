package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TurretSubsystemAaravDewan;
import org.firstinspires.ftc.teamcode.Command.DriveCommand;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TurretSubsystemAaravDewan;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.Turrettt;
import org.firstinspires.ftc.teamcode.globals.Localization;

@TeleOp(name = "TTT")
public class TTT extends CommandOpMode {

    private static final double rpm1 = 2300;
    private static final double hood1 = 0.5375;
    private static final double rpm2 = 2400.0;
    private static final double hood2 = 0.51;
    private static final double rpm3 = 3200.0;
    private static final double hood3 = 0.205;
    private static final double idlerpm = 500;

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;
    private TurretSubsystemAaravDewan turretSubsystem;
    private Turrettt turrettt;

    private Follower follower;

    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    private boolean transferOpen = false;
    private boolean flywheelEnabled = true;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystemAaravDewan(hardwareMap);

        follower = Pedropathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        turretSubsystem.aimAtRedGoal();

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        transferSubsystem.Closed();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        new DriveCommand(
                driveSubsystem,
                () -> (double) gamepad1.left_stick_y,
                () -> (double) -gamepad1.left_stick_x,
                () -> (double) - gamepad1.right_stick_x
        ).schedule();

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

        flyWheelSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (flywheelEnabled) {
                        flyWheelSubsystem.spinUp(Constants.targetRPM);
                    } else {
                        flyWheelSubsystem.stop();
                    }
                    flyWheelSubsystem.setHoodAngle(Constants.hoodAngle);
                }, flyWheelSubsystem)
        );

        turretSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    Pose p = follower.getPose();
                    turretSubsystem.update(p.getX(), p.getY(), p.getHeading());
                }, turretSubsystem)
        );

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm1, hood1);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm2, hood2);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(rpm3, hood3);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = false;
                    transferOpen = false;
                    transferSubsystem.Closed();
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = true;
                    transferOpen = true;
                    transferSubsystem.Open();
                }))
                .whenReleased(new InstantCommand(() -> {
                    transferOpen = false;
                    transferSubsystem.Closed();
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(idlerpm, hood1);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() ->
                        turretSubsystem.setAlliance(!turretSubsystem.isRedAlliance())));
    }

    private void setPreset(double rpm, double hoodPosition) {
        Constants.targetRPM = rpm;
        Constants.hoodAngle = Constants.servoPositionToHoodAngle(hoodPosition);
    }

    @Override
    public void run() {
        follower.update();
        super.run();
        Localization.update();

        turrettt.periodic();



        double currentRpmLeft = flyWheelSubsystem.getCurrentRPMLeft();
        double currentRpmRight = flyWheelSubsystem.getCurrentRPMRight();
        double currentHoodAngle = flyWheelSubsystem.getHoodAngle();
        Pose pose = follower.getPose();

        telemetry.addData("Target RPM", Constants.targetRPM);
        telemetry.addLine("----------------------------");
        telemetry.addData("Left Rpm", currentRpmLeft);
        telemetry.addData("RPM Error", Constants.targetRPM - currentRpmLeft);
        telemetry.addLine("----------------------------");
        telemetry.addData("Right Rpm", currentRpmRight);
        telemetry.addData("RPM Error", Constants.targetRPM - currentRpmRight);
        telemetry.addLine("----------------------------");
        telemetry.addData("Hood Angle", currentHoodAngle);
        telemetry.addLine("----------------------------");
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Heading deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Alliance", turretSubsystem.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addData("Turret target deg", turretSubsystem.getTargetAngleDeg());
        telemetry.addData("Turret on target", turretSubsystem.isOnTarget());
        telemetry.addData("Turret in range", turretSubsystem.isWithinRange());
        if (!turretSubsystem.isLeftPresent() || !turretSubsystem.isRightPresent()) {
            telemetry.addData("TURRET SERVO MISSING",
                    "left=" + turretSubsystem.isLeftPresent() + " right=" + turretSubsystem.isRightPresent());
        }

        telemetry.update();
    }
}