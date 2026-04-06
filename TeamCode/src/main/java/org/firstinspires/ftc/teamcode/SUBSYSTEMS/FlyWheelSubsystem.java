package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.FlyWheelConstants;

public class FlyWheelSubsystem extends SubsystemBase {

    public DcMotorEx shooterTop, shooterBottom;
    private Servo hoodServo;
    private PIDController pidTop, pidBottom;
    public VoltageSensor voltageSensor;


    //Calculate the value of ticks per rev = 28 * (flywheelSprocketTeeth / motorSprocketTeeth)
    //                                                                 (26 / 24)

    private static final double Ticks_Per_Rev = 30.33333; // this in
    private static final double maxRPM = 6000;
    private static final double maxTicks_per_rev = maxRPM * Ticks_Per_Rev/60;
    private static final double idleRPM = 1500;

    public FlyWheelSubsystem(HardwareMap hardwareMap){
        shooterTop = hardwareMap.get(DcMotorEx.class,"shooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class,"shooterBottom");
        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Change direction for both if the flywheel is spinning the wrong way
        shooterTop.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidTop = new PIDController(0,0,0);
        pidBottom = new PIDController(0,0,0);
    }

    private double rpmtoticks(double rpm){

        return rpm * Ticks_Per_Rev/60;
    }

    public void spinUp (double rpm) {
        double targetVelocity = rpmtoticks(rpm);
        double actualTop = shooterTop.getVelocity();
        double actualBottom = shooterBottom.getVelocity();

        //Update PID values every tick
        pidTop.setPID(FlyWheelConstants.kP, FlyWheelConstants.kI, FlyWheelConstants.kD);
        pidBottom.setPID(FlyWheelConstants.kP, FlyWheelConstants.kI, FlyWheelConstants.kD);

        //FeedForward
        double ff = FlyWheelConstants.kF * (targetVelocity / maxTicks_per_rev);
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

        shooterTop.setPower(powerTop);
        shooterBottom.setPower(powerBottom);

    }

    public boolean isAtSpeed(double rpm){
        double targetVelocity = rpmtoticks(rpm);
        double errorTop = Math.abs(targetVelocity - shooterTop.getVelocity());
        double errorBottom = Math.abs(targetVelocity - shooterBottom.getVelocity());
        return errorTop < FlyWheelConstants.velocityTolerance && errorBottom < FlyWheelConstants.velocityTolerance;
    }

    public void idle(){

        spinUp(idleRPM);
    }

    public void stop (){
        shooterTop.setPower(0);
        shooterBottom.setPower(0);
        pidTop.reset();
        pidBottom.reset();
    }

    public void setHoodAngle(double degrees){
        double position = FlyWheelConstants.hoodAngleToServoPosition(degrees);
        position = Math.max(
                FlyWheelConstants.hoodMinServoPosition,
                Math.min(FlyWheelConstants.hoodMaxServoPosition, position)
        );
        hoodServo.setPosition(position);
    }

    public double ticksToRPM(double ticks){

        return (ticks * 60.0 / Ticks_Per_Rev);
    }


    public double getCurrentRPMTOP() {
        return ticksToRPM(shooterTop.getVelocity());
    }

    public double getCurrentRPMBottom(){

        return ticksToRPM(shooterBottom.getVelocity());
    }
}
