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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.COMMAND.DriveCommand;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;

@TeleOp(name = "TeleOp Scrimmage Red")
public class TeleOpScrimmageRed extends CommandOpMode {

    private static final double PRESET_ONE_RPM = 2300;
    private static final double PRESET_ONE_HOOD_POSITION = FlyWheelConstants.hoodAngleToServoPosition(65);
    private static final double PRESET_TWO_RPM = 2400;
    private static final double PRESET_TWO_HOOD_POSITION = FlyWheelConstants.hoodAngleToServoPosition(65);
    private static final double PRESET_THREE_RPM = 3200.0;
    private static final double PRESET_THREE_HOOD_POSITION = FlyWheelConstants.hoodAngleToServoPosition(55);
    private static final double idlerpm = 1000;
    private static final double idleHood = 1000;
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private TransferSubsystem transferSubsystem;

    private boolean pedroDriving = false;


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

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        Pose startPose;
        if (PoseMemory.lastPose != null) {
            startPose = PoseMemory.lastPose;
        } else {
            startPose = new Pose(105.1, 132.2, Math.toRadians(90)); // fallback if teleop is run directly
        }

        follower.setStartingPose(startPose);


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
                        intakeSubsystem.runAll();
                    } else if (gamepad1.left_trigger > 0.1) {
                        intakeSubsystem.reverseAll();
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

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    flywheelEnabled = true;
                    setPreset(idlerpm, idleHood);
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

//        new GamepadButton(gamepad1Ex, GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(()-> {
//                    pedroDriving = true;
//
//                    Pose currentPose = follower.getPose();
//
//                    follower.followPath(
//                            follower.pathBuilder()
//                                    .addPath(new BezierLine
//                                            (currentPose,
//                                                    new Pose(currentPose.getX(), currentPose.getY()+1)))
//                                    .setHeadingInterpolation(
//                                            HeadingInterpolator.facingPoint(CalculatorConstants.redGoalXInches,CalculatorConstants.redGoalYInches)
//                                    )
//                                    .build(),
//                            true
//                    );
//                }));
//
//        new GamepadButton(gamepad1Ex, GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> {
//                    pedroDriving = true;
//
//                    Pose currentPose = follower.getPose();
//                    Pose gatePose = new Pose(129.2,58.096423017107305,Math.toRadians(25));
//
//                    follower.followPath(
//                            follower.pathBuilder()
//                                    .addPath(new BezierLine
//                                            (currentPose, gatePose))
//                                    .setLinearHeadingInterpolation(currentPose.getHeading(),gatePose.getHeading())
//                                    .build(),
//                            true
//                    );
//                }))
//                .whenReleased(new InstantCommand(()-> {
//                    pedroDriving = false;
//                    follower.breakFollowing();
//                    follower.startTeleOpDrive();
//                }));
//
//        new GamepadButton(gamepad1Ex, GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(() -> {
//                    pedroDriving = true;
//
//                    Pose currentPose = follower.getPose();
//                    Pose shootPoseClose = new Pose(80.33928173384129,73.05352407500875);
//
//                    follower.followPath(
//                            follower.pathBuilder()
//                                    .addPath(new BezierLine
//                                            (currentPose, shootPoseClose))
//                                    .setHeadingInterpolation(
//                                            HeadingInterpolator.facingPoint(CalculatorConstants.redGoalXInches,CalculatorConstants.redGoalXInches)
//                                    )
//                                    .build(),
//                            true
//                    );
//                }))
//                .whenReleased(new InstantCommand(()-> {
//                    pedroDriving = false;
//                    follower.breakFollowing();
//                    follower.startTeleOpDrive();
//                }));
//
//        new GamepadButton(gamepad1Ex, GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand(() -> {
//                    pedroDriving = true;
//
//                    Pose currentPose = follower.getPose();
//                    Pose shootPoseFar = new Pose(84.1796267496112,8.932348367029547);
//
//                    follower.followPath(
//                            follower.pathBuilder()
//                                    .addPath(new BezierLine
//                                            (currentPose, shootPoseFar))
//                                    .setHeadingInterpolation(
//                                            HeadingInterpolator.facingPoint(CalculatorConstants.blueGoalXInches,CalculatorConstants.blueGoalYInches)
//                                    )
//                                    .build(),
//                            true
//                    );
//                }))
//                .whenReleased(new InstantCommand(()-> {
//                    pedroDriving = false;
//                    follower.breakFollowing();
//                    follower.startTeleOpDrive();
//                }));
//
//        new GamepadButton(gamepad1Ex, GamepadKeys.Button.Y)
//                .whenPressed(new InstantCommand(() -> {
//                    pedroDriving = true;
//
//                    Pose currentPose = follower.getPose();
//                    Pose humanPlayerZoneBlue = new Pose(23.343169758724613,11.216043515133173, Math.toRadians(0));
//
//                    follower.followPath(
//                            follower.pathBuilder()
//                                    .addPath(new BezierLine
//                                            (currentPose, humanPlayerZoneBlue))
//                                    .setLinearHeadingInterpolation(currentPose.getHeading(),humanPlayerZoneBlue.getHeading())
//                                    .build(),
//                            true
//                    );
//                }))
//                .whenReleased(new InstantCommand(()-> {
//                    pedroDriving = false;
//                    follower.breakFollowing();
//                    follower.startTeleOpDrive();
//                }));
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

        follower.update();

        if (pedroDriving && !follower.isBusy()) {
            pedroDriving = false;
        }


        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));


        telemetry.addData("Target RPM", FlyWheelConstants.targetRPM);
        telemetry.addData("Top RPM", currentRpmTop);
//        telemetry.addData("RPM Error", FlyWheelConstants.targetRPM - currentRpmTop);
        telemetry.addData("Hood Angle", FlyWheelConstants.hoodAngle);
        telemetry.addData("Hood Position", currentHoodPosition);
        telemetry.addData("Flywheel Enabled", flywheelEnabled);
        telemetry.addData("Gate Open", transferOpen);
        telemetry.addData("At Speed", flyWheelSubsystem.isAtSpeed(FlyWheelConstants.targetRPM));
        telemetry.update();
    }
}
