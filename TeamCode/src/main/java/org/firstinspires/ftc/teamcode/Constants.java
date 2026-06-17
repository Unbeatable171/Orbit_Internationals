package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;//import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static volatile double hoodAngle = 65;
    public static volatile double turretAngleRad = 0.0;
    public static volatile double hoodMinServoPosition = 0.22;
    public static volatile double hoodMaxServoPosition = 1.0;
    public static volatile double hoodAngleSlope = 37.0974358974359;
    public static volatile double hoodAngleIntercept = 36.6365641025641;

    public static volatile double idleRpm = 2200;



    // Goal Poses
    public static volatile double blueGoalXInches = 0;

    public static volatile double kF = 1.4;
    public static volatile double kP = 0.001;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0;
    public static volatile double targetRPM = 900;
    public static volatile double blueGoalYInches = 142;
    public static volatile double redGoalXInches = 142;
    public static volatile double redGoalYInches = 142;

    // Vertical difference between goal center and ball release point in meters.
    public static volatile double goalHeightInches = 27;
    public static volatile double gravityInches = 386.09;
    public static volatile double scoreAngle = -45;
    public static volatile double passThroughRadius = 5;

    // Hood angles are physical launch angles in degrees, not servo positions.
    public static volatile double minHoodAngleDeg = 44.798;
    public static volatile double maxHoodAngleDeg = 73.734;


    //TODO Retune rpm vs velocity regression, with constant hood angle

    // Exit velocity -> RPM regression coefficients.
    public static volatile double rpmA = 0;
    public static volatile double rpmB = 0 ;
    public static volatile double rpmC = 0;
    public static volatile double rpmD = 0;
    public static volatile double rpmE = 0;


    // Valid shooting limits.
    public static volatile double minDistanceToGoalMeters = 0.4;
    public static volatile double maxDistanceToGoalMeters = 5.0;
    public static volatile double minRpm = 0.0;
    public static volatile double maxRpm = 6000.0;


    public static volatile double releaseOffsetXInches = -3.14961;
    public static volatile double releaseOffsetYInches = 0.0;

    // Turret calibration.
    public static volatile double turretServo1Zero = 0.5;
    public static volatile double turretServo2Zero = 0.5;
    public static volatile double turretMinServoPosition = 0.0;
    public static volatile double turretMaxServoPosition = 1.0;
    public static volatile double turretDegreesPerServoPosition = 130.17;
    public static volatile double turretAngleOffsetDeg = 0.0;
    public static volatile boolean turretServo2Reversed = true;
    public static volatile double turretLeadScale = 1.0;

    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

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



}
