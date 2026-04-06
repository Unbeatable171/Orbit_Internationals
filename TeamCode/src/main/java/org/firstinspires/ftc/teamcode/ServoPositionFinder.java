package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ServoPositionFinder")
public class ServoPositionFinder extends OpMode {

    private Servo hoodServo, gateServo;
    private double hoodposition = 0.1;
    private double gateposition = 0.32;
    private double increment = 0.02;

    private boolean prevUp, prevDown, prevRight, prevLeft;


    @Override
    public void init () {
        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
        gateServo = hardwareMap.get(Servo.class,"transfer");
    }


    @Override
    public void loop() {

        if (gamepad1.dpad_up && !prevUp){
            hoodposition += increment;
        }
        if(gamepad1.dpad_down && !prevDown){
            hoodposition-= increment;
        }

        hoodposition = Range.clip(
                hoodposition,
                FlyWheelConstants.hoodMinServoPosition,
                FlyWheelConstants.hoodMaxServoPosition
        );
        hoodServo.setPosition(hoodposition);

        if(gamepad1.dpad_right && !prevRight){
            gateposition += increment;
        }
        if(gamepad1.dpad_left && !prevLeft){
            gateposition -= increment;
        }

        gateposition = Range.clip(gateposition, 0.0, 1.0);
        gateServo.setPosition(gateposition);

        telemetry.addData("Hood position is",hoodposition);
        telemetry.addData("Hood angle is", FlyWheelConstants.servoPositionToHoodAngle(hoodposition));
        telemetry.addData("Increment is",increment);
        telemetry.addData("Gate position is", gateposition);
        telemetry.addData("Gate angle is", gateposition*300);
        telemetry.update();

        prevLeft = gamepad1.dpad_left;
        prevRight = gamepad1.dpad_right;
        prevDown = gamepad1.dpad_down;
        prevUp = gamepad1.dpad_up;


    }
}
