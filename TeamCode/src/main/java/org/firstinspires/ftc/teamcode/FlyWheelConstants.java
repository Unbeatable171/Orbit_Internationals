package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FlyWheelConstants{
    public static double kF = 1.215;
    public static double kP = 0.00215;
    public static double kI = 0.0;
    public static double kD = 0.00015;
    public static double velocityTolerance = 50.0;
    public static double targetRPM = 2450;
    public static double hoodAngle = 60;
    public static double hoodMinServoPosition = 0;
    public static double hoodMaxServoPosition = 1.0;
    public static double hoodAngleSlope = 46.11;
    public static double hoodAngleIntercept = 36.89;

    // Calibrated launch-angle fit: outputAngleDeg = m * servoPosition + b.

    public static double hoodMinAngleDeg() {
        return servoPositionToHoodAngle(hoodMinServoPosition);
    }

    public static double hoodMaxAngleDeg() {
        return servoPositionToHoodAngle(hoodMaxServoPosition);
    }

    public static double hoodAngleToServoPosition(double hoodAngleDeg) {
        return (hoodAngleDeg - hoodAngleIntercept) / hoodAngleSlope;
    }

    public static double servoPositionToHoodAngle(double servoPosition) {
        return hoodAngleSlope * servoPosition + hoodAngleIntercept;
    }

}
