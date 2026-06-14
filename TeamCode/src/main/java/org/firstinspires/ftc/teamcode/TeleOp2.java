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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;
import org.firstinspires.ftc.teamcode.COMMAND.DriveCommand;

@TeleOp(name = "TeleOp2")
public class TeleOp2 extends CommandOpMode {

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
                    flyWheelSubsystem.setTurretAngleRad(Constants.turretAngleRad);
                }, flyWheelSubsystem)
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

    }

    private void setPreset(double rpm, double hoodPosition) {
        Constants.targetRPM = rpm;
        Constants.hoodAngle = Constants.servoPositionToHoodAngle(hoodPosition);
    }

    @Override
    public void run() {
        super.run();

        double currentRpmLeft = flyWheelSubsystem.getCurrentRPMLeft();
        double currentRpmRight = flyWheelSubsystem.getCurrentRPMRight();
        double currentHoodAngle = flyWheelSubsystem.getHoodAngle();

        telemetry.addData("Target RPM", Constants.targetRPM);
        telemetry.addLine("----------------------------");
        telemetry.addData("Left Rpm",currentRpmLeft);
        telemetry.addData("RPM Error", Constants.targetRPM - currentRpmLeft);
        telemetry.addLine("----------------------------");
        telemetry.addData("Right Rpm", currentRpmRight);
        telemetry.addData("RPM Error", Constants.targetRPM - currentRpmRight);
        telemetry.addLine("----------------------------");
        telemetry.addData("Hood Angle", currentHoodAngle);

        telemetry.update();
    }


}
