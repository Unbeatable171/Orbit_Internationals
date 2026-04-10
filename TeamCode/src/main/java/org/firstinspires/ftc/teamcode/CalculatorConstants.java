package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CalculatorConstants {
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    //TODO Retune the values

    // Field target positions in meters, using the same field reference as Pedro after conversion.
    public static double blueGoalXInches = 0;
    public static double blueGoalYInches = 138;
    public static double redGoalXInches = 138;
    public static double redGoalYInches = 138;

    // Vertical difference between goal center and ball release point in meters.
    public static double goalHeightInches = 0;
    public static double gravityInches = 386.09;
    public static double scoreAngle = Math.toRadians(-30);
    public static double passThroughRadius = 5;
    // Hood angles are physical launch angles in degrees, not servo positions.
    public static double minHoodAngleDeg = 41.5;
    public static double maxHoodAngleDeg = 83.0;
    public static double defaultHoodAngleDeg = 60.0;

   public double hoodAngletoServo(){
       return;
   }

    //TODO Retune rpm vs velocity regression, with constant hood angle

    // Exit velocity -> RPM regression coefficients.
    public static double rpmA = 0.0;
    public static double rpmB = 0.0;
    public static double rpmC = 0.0;


    // Valid shooting limits.
    public static double minDistanceToGoalMeters = 0.4;
    public static double maxDistanceToGoalMeters = 5.0;
    public static double minRpm = 0.0;
    public static double maxRpm = 6000.0;


    // Fallback shot presets by triangle zone.
    // Close fallback triangle matches the first shooting triangle used in close autos.
    public static double closeFallbackTriangleAXInches = 72.0;
    public static double closeFallbackTriangleAYInches = 72.0;
    public static double closeFallbackTriangleBXInches = 0.0;
    public static double closeFallbackTriangleBYInches = 144.0;
    public static double closeFallbackTriangleCXInches = 144.0;
    public static double closeFallbackTriangleCYInches = 144.0;
    public static double closeFallbackHoodAngleDeg = FlyWheelConstants.servoPositionToHoodAngle(0.55);
    public static double closeFallbackRpm = 2300.0;

    // Far fallback triangle matches the second shooting triangle used in autos.
    public static double farFallbackTriangleAXInches = 72.0;
    public static double farFallbackTriangleAYInches = 23.0;
    public static double farFallbackTriangleBXInches = 95.0;
    public static double farFallbackTriangleBYInches = 0.0;
    public static double farFallbackTriangleCXInches = 49.0;
    public static double farFallbackTriangleCYInches = 0.0;
    public static double farFallbackHoodAngleDeg = FlyWheelConstants.servoPositionToHoodAngle(0.275);
    public static double farFallbackRpm = 3200.0;



//    // Heading lock tuning. The P terms match the primary/secondary Pedro heading PIDFs.
    public static double headingPrimaryKp = 0.8;
    public static double headingSecondaryKp = 0.8;
    public static double headingSecondaryThresholdDeg = 6.0;
    public static double maxTurnPower = 0.5;
    public static double headingToleranceDeg = 3.0;

    // Release point offset from robot pose origin in meters.
    // X is along the robot's forward/back axis (field length in heading direction).
    // A negative X offset means the release point is behind the robot center.
    public static double releaseOffsetXInches = -3.14961;
    public static double releaseOffsetYInches = 0.0;

//    // Far-shot transfer pulsing to let the flywheel recover between shots.
//    public static double farShotPauseSeconds = 0.3;
//    public static double farShotFeedSeconds = 0.12;
//    public static double farShotDistanceThresholdMeters = 2.715;
//    public static int farShotPulseCount = 3;
}
