package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CONSTANTS;

public class FlyWheelSubsystemUpdated extends SubsystemBase {

    public DcMotorEx shooterLeft, shooterRight;
    private Servo hoodServo;
    private Servo turretServo1, turretServo2;
    private final HardwareMap hardwareMap;
    private PIDController pidTop, pidBottom;
    public VoltageSensor voltageSensor;



    private static final double Ticks_Per_Rev = 28; // this in
    private static final double maxRPM = 6000;
    private static final double maxTicks_per_rev = maxRPM * Ticks_Per_Rev/60;
    private static final double idleRPM = 2000;

    public FlyWheelSubsystemUpdated(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        shooterLeft = hardwareMap.get(DcMotorEx.class,"shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class,"shooterRight");
        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Change direction for both if the flywheel is spinning the wrong way
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidTop = new PIDController(0,0,0);
        pidBottom = new PIDController(0,0,0);
    }

    private double rpmtoticks(double rpm){

        return rpm * Ticks_Per_Rev/60;
    }
    public void spinUp (double rpm) {
        //mmm
        //if(Double.isNaN(rpm) || rpm < 0) return;
        if ( rpm < 3000) {
            double targetVelocity = rpmtoticks(rpm);
            double actualTop = shooterLeft.getVelocity();
            double actualBottom = shooterLeft.getVelocity();

            //Update PID values every tick
            pidTop.setPID(CONSTANTS.kP, CONSTANTS.kI, CONSTANTS.kD);
            pidBottom.setPID(CONSTANTS.kP, CONSTANTS.kI, CONSTANTS.kD);

            //FeedForward
            double ff = CONSTANTS.kF * (targetVelocity / maxTicks_per_rev);
            // FlyWheelConstants.kS + FlyWheelConstants.kV * (targetVelocity / maxTicks_per_rev);

            //Voltage Compensation
            double voltageScale = 12.0 / voltageSensor.getVoltage();

            //Clamp
            double powerTop = ff + pidTop.calculate(actualTop, targetVelocity);
            double powerBottom = ff + pidBottom.calculate(actualBottom, targetVelocity);

            powerTop *= voltageScale;
            powerBottom *= voltageScale;

            powerTop = Math.max(-1, Math.min(1, powerTop));
            powerBottom = Math.max(-1, Math.min(1, powerBottom));

            shooterLeft.setPower(powerTop);
            shooterRight .setPower(powerBottom);
        }
        else {
            double targetVelocity = rpmtoticks(rpm);
            double actualTop = shooterLeft.getVelocity();
            double actualBottom = shooterLeft.getVelocity();

            //Update PID values every tick
            pidTop.setPID(CONSTANTS.kPHigh, CONSTANTS.kIHigh, CONSTANTS.kDHigh);
            pidBottom.setPID(CONSTANTS.kPHigh, CONSTANTS.kIHigh, CONSTANTS.kDHigh);

            //FeedForward
            double ff = CONSTANTS.kFHigh * (targetVelocity / maxTicks_per_rev);
            // FlyWheelConstants.kS + FlyWheelConstants.kV * (targetVelocity / maxTicks_per_rev);

            //Voltage Compensation
            double voltageScale = 12.0 / voltageSensor.getVoltage();

            //Clamp
            double powerTop = ff + pidTop.calculate(actualTop, targetVelocity);
            double powerBottom = ff + pidBottom.calculate(actualBottom, targetVelocity);

            powerTop *= voltageScale;
            powerBottom *= voltageScale;

            powerTop = Math.max(-1, Math.min(1, powerTop));
            powerBottom = Math.max(-1, Math.min(1, powerBottom));

            shooterLeft.setPower(powerTop);
            shooterRight.setPower(powerBottom);
        }

    }


    public void idle(){
        spinUp(idleRPM);
    }

    public void stop (){
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        pidTop.reset();
        pidBottom.reset();
    }

    public void setHoodAngle(double degrees){
        double position = CONSTANTS.hoodAngleToServoPosition(degrees);
        position = Math.max(
                CONSTANTS.hoodMinServoPosition,
                Math.min(CONSTANTS.hoodMaxServoPosition, position)
        );
        hoodServo.setPosition(position);
    }

    public double getHoodAngle() {
        return CONSTANTS.servoPositionToHoodAngle(hoodServo.getPosition());
    }



    public double ticksToRPM(double ticks){

        return (ticks * 60.0 / Ticks_Per_Rev);
    }


    public double getCurrentRPMLeft() {
        return ticksToRPM(shooterLeft.getVelocity());
    }

    public double getCurrentRPMRight(){

        return ticksToRPM(shooterRight.getVelocity());
    }


}
