package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double kF = 1;
    public static double kP = 1;
    public static double kI = 0.0;
    public static double kD = 0;
    public static double targetRPM = 1500;
    public static double hoodAngle = 65;
    public static double turretAngleRad = 0.0;
    public static double hoodMinServoPosition = 0.28;
    public static double hoodMaxServoPosition = 1.0;
    public static double hoodAngleSlope = 36.67;
    public static double hoodAngleIntercept = 38.33;

    public static double idleRpm = 2200;


    public static double hoodAngleToServoPosition(double hoodAngleDeg) {
        return (hoodAngleDeg - hoodAngleIntercept) / hoodAngleSlope;
    }

    public static double servoPositionToHoodAngle(double servoPosition) {
        return (hoodAngleSlope * servoPosition) + hoodAngleIntercept;
    }

    //--------------------------------------------------------------------//

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }


    // Goal Poses
    public static double blueGoalXInches = 0;
    public static double blueGoalYInches = 142;
    public static double redGoalXInches = 142;
    public static double redGoalYInches = 142;

    // Vertical difference between goal center and ball release point in meters.
    public static double goalHeightInches = 27;
    public static double gravityInches = 386.09;
    public static double scoreAngle = -45;
    public static double passThroughRadius = 5;

    // Hood angles are physical launch angles in degrees, not servo positions.
    public static double minHoodAngleDeg = 48.6;
    public static double maxHoodAngleDeg = 75;


    //TODO Retune rpm vs velocity regression, with constant hood angle

    // Exit velocity -> RPM regression coefficients.
    public static double rpmA = 0;
    public static double rpmB = 0 ;
    public static double rpmC = 0;
    public static double rpmD = 0;
    public static double rpmE = 0;


    // Valid shooting limits.
    public static double minDistanceToGoalMeters = 0.4;
    public static double maxDistanceToGoalMeters = 5.0;
    public static double minRpm = 0.0;
    public static double maxRpm = 6000.0;


    public static double releaseOffsetXInches = -3.14961;
    public static double releaseOffsetYInches = 0.0;

    // Turret calibration.
    public static double turretServo1Zero = 0.5;
    public static double turretServo2Zero = 0.5;
    public static double turretMinServoPosition = 0.0;
    public static double turretMaxServoPosition = 1.0;
    public static double turretDegreesPerServoPosition = 130.17;
    public static double turretAngleOffsetDeg = 0.0;
    public static boolean turretServo2Reversed = true;
    public static double turretLeadScale = 1.0;

    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
