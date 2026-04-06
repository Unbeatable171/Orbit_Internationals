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
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;

@TeleOp(name = "TeleOp 1")
public class TELEOP1 extends CommandOpMode {

    private static final double PRESET_ONE_RPM = 2300;
    private static final double PRESET_ONE_HOOD_POSITION = 0.5375;
    private static final double PRESET_TWO_RPM = 2200.0;
    private static final double PRESET_TWO_HOOD_POSITION = 0.51;
    private static final double PRESET_THREE_RPM = 3200.0;
    private static final double PRESET_THREE_HOOD_POSITION = 0.205;

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

        setPreset(PRESET_ONE_RPM, PRESET_ONE_HOOD_POSITION);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        new DriveCommand(
                driveSubsystem,
                () -> (double) -gamepad1.left_stick_y,
                () -> (double) -gamepad1.right_stick_x,
                () -> (double) -gamepad1.left_stick_x
        ).schedule();

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

        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    if (transferOpen) {
                        intakeSubsystem.transfer();
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeSubsystem.on();
                    } else if (gamepad1.left_trigger > 0.1) {
                        intakeSubsystem.reverse();
                    } else {
                        intakeSubsystem.off();
                    }
                }, intakeSubsystem)
        );

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(PRESET_ONE_RPM, PRESET_ONE_HOOD_POSITION);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(PRESET_TWO_RPM, PRESET_TWO_HOOD_POSITION);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    setPreset(PRESET_THREE_RPM, PRESET_THREE_HOOD_POSITION);
                    flywheelEnabled = true;
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = false;
                    transferOpen = false;
                    transferSubsystem.Closed();
                }));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = true;
                    transferOpen = true;
                    transferSubsystem.Open();
                }))
                .whenReleased(new InstantCommand(() -> {
                    transferOpen = false;
                    transferSubsystem.Closed();
                }));
    }

    private void setPreset(double rpm, double hoodPosition) {
        FlyWheelConstants.targetRPM = rpm;
        FlyWheelConstants.hoodAngle = FlyWheelConstants.servoPositionToHoodAngle(hoodPosition);
    }

    @Override
    public void run() {
        super.run();

        double currentRpmTop = flyWheelSubsystem.getCurrentRPMTOP();
        double currentHoodPosition = FlyWheelConstants.hoodAngleToServoPosition(FlyWheelConstants.hoodAngle);

        telemetry.addData("Target RPM", FlyWheelConstants.targetRPM);
        telemetry.addData("Top RPM", currentRpmTop);
        telemetry.addData("RPM Error", FlyWheelConstants.targetRPM - currentRpmTop);
        telemetry.addData("Hood Angle", FlyWheelConstants.hoodAngle);
        telemetry.addData("Hood Position", currentHoodPosition);
        telemetry.addData("Flywheel Enabled", flywheelEnabled);
        telemetry.addData("Gate Open", transferOpen);
        telemetry.addData("At Speed", flyWheelSubsystem.isAtSpeed(FlyWheelConstants.targetRPM));
        telemetry.update();
    }
}
