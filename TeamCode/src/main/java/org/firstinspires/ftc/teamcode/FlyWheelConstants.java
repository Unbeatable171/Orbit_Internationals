package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FlyWheelConstants{
    public static double kF = 1.131;
    public static double kP = 0.0085;
    public static double kI = 0.0;
    public static double kD = 0.00012;

    public static double kFHigh = 1.131;
    public static double kPHigh = 0.0045;
    public static double kIHigh = 0.0;
    public static double kDHigh = 0.0002;
    public static double velocityTolerance = 50.0;
    public static double targetRPM = 2300;
    public static double hoodAngle = 65;
    public static double hoodMinServoPosition = 0.1;
    public static double hoodMaxServoPosition = 1.0;
    public static double hoodAngleSlope = 36.67;
    public static double hoodAngleIntercept = 38.33;

    public static double velocityoffset = 150;
    public static double rpmoffset = 0;
    public static double rpmoffsetFar = 350;
    public static double idleRpm = 2200;
    public static double rpmVelocitySlope = 12.9285;
    public static double rpmVelocityIntercept = -268.42;

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
        return (hoodAngleSlope * servoPosition) + hoodAngleIntercept;
    }

}
