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

import org.firstinspires.ftc.teamcode.COMMAND.DriveCommand;
import org.firstinspires.ftc.teamcode.COMMAND.FlyWheelCommand;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.*;
import org.firstinspires.ftc.teamcode.FlyWheelConstants;

@TeleOp(name = "TeleOp Test 1")
public class TELEOPTEST extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;

    private GamepadEx gamepadEx;

    private double rpmIncrement = 50;
    private double angleIncrement = 2;

    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean gateOpen = false;
    private boolean flywheelEnabled = true;

    @Override
    public void initialize() {

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        transferSubsystem.Closed();
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);

        gamepadEx = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        // ---------------- DRIVE ----------------
        new DriveCommand(
                driveSubsystem,
                () -> (double) -gamepad1.left_stick_y,
                () -> (double) -gamepad1.right_stick_x,
                () -> (double) -gamepad1.left_stick_x
        ).schedule();

        // ---------------- FLYWHEEL DEFAULT CONTROL ----------------
        flyWheelSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (flywheelEnabled) {
                        flyWheelSubsystem.spinUp(FlyWheelConstants.targetRPM);
                    } else {
                        flyWheelSubsystem.stop();
                    }
                    flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);
                }, flyWheelSubsystem)
        );

        new GamepadButton(gamepadEx, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = true));

        new GamepadButton(gamepadEx, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> flywheelEnabled = false));


        // ---------------- TRANSFER BUTTON ----------------
        new GamepadButton(gamepadEx, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    gateOpen = true;
                    transferSubsystem.Open();
                })
                .whenReleased(() -> {
                    gateOpen = false;
                    transferSubsystem.Closed();
                });

        //----------------INTAKE CONTROL -----------------

        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (gateOpen) {
                        intakeSubsystem.transfer();
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeSubsystem.on();
                    } else if (gamepad1.left_trigger > 0.1) {
                        intakeSubsystem.reverse();
                    } else {
                        intakeSubsystem.off();
                    }
                }, intakeSubsystem));


        // ---------------- MAIN CONTROL LOOP ----------------
        new RunCommand(() -> {

            // ---- DPAD TUNING ----
//            if (gamepad1.dpad_up && !prevDpadUp) FlyWheelConstants.targetRPM += rpmIncrement;
//            if (gamepad1.dpad_down && !prevDpadDown) FlyWheelConstants.targetRPM -= rpmIncrement;
            if (gamepad1.dpad_right && !prevDpadRight) FlyWheelConstants.hoodAngle += angleIncrement;
            if (gamepad1.dpad_left && !prevDpadLeft) FlyWheelConstants.hoodAngle -= angleIncrement;

            FlyWheelConstants.targetRPM = Math.max(0, Math.min(6000, FlyWheelConstants.targetRPM));
            FlyWheelConstants.hoodAngle = Math.max(
                    FlyWheelConstants.hoodMinAngleDeg(),
                    Math.min(FlyWheelConstants.hoodMaxAngleDeg(), FlyWheelConstants.hoodAngle)
            );

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;

        }).schedule();

    }



    @Override
    public void run() {
        super.run();
        flyWheelSubsystem.setHoodAngle(FlyWheelConstants.hoodAngle);

        double currentRPM = flyWheelSubsystem.getCurrentRPMTOP();
        double currentRPMBottom = flyWheelSubsystem.getCurrentRPMBottom();
        double currenthoodposition = FlyWheelConstants.hoodAngleToServoPosition(FlyWheelConstants.hoodAngle);
        double currentoutangle = FlyWheelConstants.servoPositionToHoodAngle(currenthoodposition);

        telemetry.addData("Target RPM", FlyWheelConstants.targetRPM);
        telemetry.addData("Top RPM", currentRPM);
        telemetry.addData("BottomRPM", currentRPMBottom);
        telemetry.addData("ticks/sec", flyWheelSubsystem.shooterTop.getVelocity());
        telemetry.addData("RPM Error", FlyWheelConstants.targetRPM - currentRPM);

        telemetry.addData("Out Angle", currentoutangle);
        telemetry.addData("Servo Position", currenthoodposition);


        telemetry.addData("Hood Angle", FlyWheelConstants.hoodAngle);
        telemetry.addData("At Speed", flyWheelSubsystem.isAtSpeed(FlyWheelConstants.targetRPM));
        telemetry.addData("Flywheel Enabled", flywheelEnabled);
        telemetry.addData("Current Voltage",flyWheelSubsystem.voltageSensor.getVoltage());

        telemetry.update();

    }
}
