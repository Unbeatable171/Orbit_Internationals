package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double kF = 1.131;
    public static double kP = 0.0085;
    public static double kI = 0.0;
    public static double kD = 0.00012;
    public static double targetRPM = 2300;
    public static double hoodAngle = 65;
    public static double hoodMinServoPosition = 0.28;
    public static double hoodMaxServoPosition = 1.0;
    public static double hoodAngleSlope = 36.67;
    public static double hoodAngleIntercept = 38.33;

    public static double idleRpm = 2200;
    public static double rpmVelocitySlope = 0;
    public static double rpmVelocityIntercept = 0;


    public static double hoodAngleToServoPosition(double hoodAngleDeg) {
        return (hoodAngleDeg - hoodAngleIntercept) / hoodAngleSlope;
    }

    public static double servoPositionToHoodAngle(double servoPosition) {
        return (hoodAngleSlope * servoPosition) + hoodAngleIntercept;
    }
}
