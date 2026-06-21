package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

@TeleOp(name = "Turret Debug")
public class TurretDebug extends LinearOpMode {

    private DcMotor motor;
    private DcMotor motor1;

    private ServoEx servoRightBack;
    private ServoEx servoLeftBack;
    private ServoExGroup turretServos;

    private double motorPower = 1.0;
    private double servoPos = 0.5;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(
                DcMotor.class,
                "shooterRight"
        );

        motor1 = hardwareMap.get(
                DcMotor.class,
                "shooterLeft"
        );

        servoRightBack = new ServoEx(
                hardwareMap,
                "turretservoright"
        );

        servoLeftBack = new ServoEx(
                hardwareMap,
                "turretservoleft"
        );


        PwmControl.PwmRange turretPwmRange =
                new PwmControl.PwmRange(500, 2500);

        servoRightBack.setPwm(turretPwmRange);
        servoLeftBack.setPwm(turretPwmRange);

        turretServos = new ServoExGroup(
                servoRightBack,
                servoLeftBack
        );


        servoRightBack.setInverted(false);
        servoLeftBack.setInverted(false);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turretServos.set(servoPos);

        telemetry.addLine("Manual test initialized");
        telemetry.addData("Starting Servo Position", servoPos);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {

                motor.setPower(motorPower);
                motor1.setPower(-motorPower);

            } else if (gamepad1.b) {

                motor.setPower(-motorPower);
                motor1.setPower(motorPower);

            } else {

                motor.setPower(0);
                motor1.setPower(0);
            }

            if (gamepad1.x) {
                servoPos -= 0.0025;
                sleep(40);
            }

            if (gamepad1.y) {
                servoPos += 0.0025;
                sleep(40);
            }


            if (gamepad1.dpad_left) {
                servoPos = 0.4;
            }

            if (gamepad1.dpad_up) {
                servoPos = 0.5;
            }

            if (gamepad1.dpad_right) {
                servoPos = 0.6;
            }


            servoPos = clamp(
                    servoPos,
                    0.0,
                    1.0
            );

            turretServos.set(servoPos);


            if (gamepad1.left_bumper) {
                motor.setDirection(
                        DcMotor.Direction.FORWARD
                );
            }

            if (gamepad1.right_bumper) {
                motor.setDirection(
                        DcMotor.Direction.REVERSE
                );
            }

            telemetry.addData(
                    "Shooter Right Power",
                    motor.getPower()
            );

            telemetry.addData(
                    "Shooter Left Power",
                    motor1.getPower()
            );

            telemetry.addData(
                    "Shooter Right Encoder",
                    motor.getCurrentPosition()
            );

            telemetry.addData(
                    "Shooter Left Encoder",
                    motor1.getCurrentPosition()
            );

            telemetry.addData(
                    "Shooter Right Direction",
                    motor.getDirection()
            );

            telemetry.addData(
                    "Turret Servo Position",
                    servoPos
            );

            telemetry.addData(
                    "Right Servo Inverted",
                    servoRightBack.getInverted()
            );

            telemetry.addData(
                    "Left Servo Inverted",
                    servoLeftBack.getInverted()
            );

            telemetry.update();
        }

        motor.setPower(0);
        motor1.setPower(0);
    }

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}
