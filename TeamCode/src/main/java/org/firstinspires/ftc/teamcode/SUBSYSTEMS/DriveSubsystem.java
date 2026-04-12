package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    public DcMotorEx frontleft, frontright, backleft, backright;

    private static final double MAX_TICKS_PER_SECOND = 2503;
    private static final double DEADZONE = 0.001;

    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    private Double headingOverride = null;

    public void setHeadingOverride(double turn) {
        this.headingOverride = turn;
    }

    public void clearHeadingOverride() {
        this.headingOverride = null;
    }

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontleft  = hardwareMap.get(DcMotorEx.class, "fl");
        frontright = hardwareMap.get(DcMotorEx.class, "fr");
        backleft   = hardwareMap.get(DcMotorEx.class, "bl");
        backright  = hardwareMap.get(DcMotorEx.class, "br");

        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double rotate) {

        forward = applyDeadzone(forward, DEADZONE);
        strafe  = applyDeadzone(strafe,  DEADZONE);
        rotate  = applyDeadzone(rotate,  DEADZONE);



        forward = Math.pow(forward, 3);
        strafe  = Math.pow(strafe,  3);
        rotate  = Math.pow(rotate,  3) *0.8;

        if (headingOverride != null) strafe = headingOverride;

        double fl = forward + strafe + rotate;
        double bl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }

        frontleft.setPower(fl);
        frontright.setPower(fr);
        backleft.setPower(bl);
        backright.setPower(br);
    }
}
