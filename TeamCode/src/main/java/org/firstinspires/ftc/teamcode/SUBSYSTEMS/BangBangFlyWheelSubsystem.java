package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CONSTANTS;

@Config
public class BangBangFlyWheelSubsystem extends SubsystemBase {

    public static volatile double bangBangThresholdRPM = 100.0;
    public static volatile double bangBangOnPower = 1.0;
    public static volatile double bangBangOffPower = 0.0;
    public static volatile double idleRPM = 2000.0;

    public DcMotorEx shooterLeft, shooterRight;
    private final Servo hoodServo;

    private static final double TICKS_PER_REV = 28.0;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public BangBangFlyWheelSubsystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinUp(double rpm) {
        if (Double.isNaN(rpm) || rpm <= 0.0) {
            stop();
            return;
        }

        leftPower = calculateBangBangPower(getCurrentRPMLeft(), rpm, leftPower);
        rightPower = calculateBangBangPower(getCurrentRPMRight(), rpm, rightPower);

        shooterLeft.setPower(leftPower);
        shooterRight.setPower(rightPower);
    }

    public void idle() {
        spinUp(idleRPM);
    }

    public void stop() {
        leftPower = 0.0;
        rightPower = 0.0;
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    public void setHoodAngle(double degrees) {
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

    public double ticksToRPM(double ticks) {
        return ticks * 60.0 / TICKS_PER_REV;
    }

    public double getCurrentRPMLeft() {
        return ticksToRPM(shooterLeft.getVelocity());
    }

    public double getCurrentRPMRight() {
        return ticksToRPM(shooterRight.getVelocity());
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    private double calculateBangBangPower(double currentRPM, double targetRPM, double currentPower) {
        if (currentRPM < targetRPM - bangBangThresholdRPM) {
            return bangBangOnPower;
        }
        if (currentRPM > targetRPM + bangBangThresholdRPM) {
            return bangBangOffPower;
        }
        return currentPower;
    }
}
