package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;//import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
@Config
public class CONSTANTS {

    public static volatile double hoodAngle = 50;
    public static volatile double turretAngleRad = 0.0;
    public static volatile double hoodMinServoPosition = 0.2;
    public static volatile double hoodMaxServoPosition = 1.0;
    public static volatile double hoodAngleSlope = 37.0974358974359;
    public static volatile double hoodAngleIntercept = 36.6365641025641;


    // ── Bang-Bang tuning ──────────────────────────────────────────
    private static final double THRESHOLD_RPM  = 100.0; // deadband ± RPM

    // ─────────────────────────────────────────────────────────────

    public static volatile double idleRpm = 2200;



    // Goal Poses
    public static volatile double blueGoalXInches = 0;

    public static volatile double kF = 1.37;
    public static volatile double kP = 0.006;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0;
    public static volatile double targetRPM = 0;
    public static volatile double blueGoalYInches = 116;
    public static volatile double redGoalXInches = 142;
    public static volatile double redGoalYInches = 142;

    // Vertical difference between goal center and ball release point in meters.
    public static volatile double goalHeightInches = 22;
    public static volatile double gravityInches = 386.09;
    public static volatile double scoreAngle = -32;
    public static volatile double passThroughRadius = 5;

    // Hood angles are physical launch angles in degrees, not servo positions.
    public static volatile double minHoodAngleDeg = 44.798;
    public static volatile double maxHoodAngleDeg = 73.734;


    //TODO Retune rpm vs velocity regression, with constant hood angle

    // Exit velocity -> RPM regression coefficients.
    public static volatile double rpmA = 0.0369772;
    public static volatile double rpmB = -2.58908 ;
    public static volatile double rpmC = 1701.90631;
    public static volatile double rpmD = 0;
    public static volatile double rpmE = 0;


    // Valid shooting limits.
    public static volatile double minDistanceToGoalMeters = 0.4;
    public static volatile double maxDistanceToGoalMeters = 5.0;
    public static volatile double minRpm = 0.0;
    public static volatile double maxRpm = 6000.0;


    public static volatile double releaseOffsetXInches = 0;
    public static volatile double releaseOffsetYInches = -1.5;

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
