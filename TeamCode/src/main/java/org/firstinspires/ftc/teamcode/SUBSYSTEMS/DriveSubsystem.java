package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    public DcMotorEx frontleft, frontright, backleft, backright;


    public DriveSubsystem(HardwareMap hardwareMap) {
        frontleft  = hardwareMap.get(DcMotorEx.class, "fl");
        frontright = hardwareMap.get(DcMotorEx.class, "fr");
        backleft   = hardwareMap.get(DcMotorEx.class, "bl");
        backright  = hardwareMap.get(DcMotorEx.class, "br");

        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double rotate) {

//        old one -
//        forward = Math.pow(forward, 3);
//        strafe  = Math.pow(strafe,  3) * 0.8;
//        rotate  = Math.pow(rotate,  3);

        forward = Math.pow(forward, 1);
        strafe  = Math.pow(strafe,  1);
        rotate  = Math.pow(rotate,  1); //*0.8


        double fl = forward + strafe + rotate;
        double bl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (max > 1.0)
        {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;}

        frontleft.setPower(fl);
        frontright.setPower(fr);
        backleft.setPower(bl);
        backright.setPower(br);


    }
}
