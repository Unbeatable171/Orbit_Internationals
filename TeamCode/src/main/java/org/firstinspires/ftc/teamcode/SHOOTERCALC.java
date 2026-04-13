package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

public class SHOOTERCALC {

    public enum GoalTarget {
        BLUE,
        RED
    }

    private final double goalXInches = CalculatorConstants.redGoalXInches;
    private final double goalYInches = CalculatorConstants.redGoalYInches;

    private double g = CalculatorConstants.gravityInches;
    private double y = CalculatorConstants.goalHeightInches;
    private double a = CalculatorConstants.scoreAngle;

    public double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    public double releaseXInches(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotX
                + CalculatorConstants.releaseOffsetXInches * cos
                - CalculatorConstants.releaseOffsetYInches * sin;
    }

    public double releaseYInches(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotY
                + CalculatorConstants.releaseOffsetXInches * sin
                + CalculatorConstants.releaseOffsetYInches * cos;
    }

    public double distanceToGoalInches(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);

        double distanceX = goalXInches - releaseX;
        double distanceY = goalYInches - releaseY;

        return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    }

    public double hoodAngleCalculator(double distanceInches) {
        double hoodAngleDeg = Math.toDegrees(
                Math.atan((2 * y / distanceInches) - Math.tan(Math.toRadians(a)))
        );
        return clamp(
                hoodAngleDeg,
                CalculatorConstants.minHoodAngleDeg,
                CalculatorConstants.maxHoodAngleDeg
        );
    }

    public double velocityCalculator(double distanceMeters, double hoodAngleDeg) {
        double x = distanceMeters;
        double y = CalculatorConstants.goalHeightInches;
        double angleRad = Math.toRadians(hoodAngleDeg);

        double cos = Math.cos(angleRad);
        double tan = Math.tan(angleRad);
        double denominator = 2 * cos * cos * (x * tan - y);


        return Math.sqrt(
                (CalculatorConstants.gravityInches * x * x) / denominator
        );
    }

    public double rpmCalculator(double velocityInchesPerSec) {
        double rpm = CalculatorConstants.rpmA * velocityInchesPerSec * velocityInchesPerSec * velocityInchesPerSec
                + CalculatorConstants.rpmB * velocityInchesPerSec * velocityInchesPerSec
                + CalculatorConstants.rpmC* velocityInchesPerSec
                + CalculatorConstants.rpmD;

        return clamp(rpm, CalculatorConstants.minRpm, CalculatorConstants.maxRpm);
    }

    public ShooterCalculatorRed.ShotSolution calculateShotSolution(double robotX, double robotY,
                                                                   double robotHeadingRad,
                                                                   double robotVelocityXInchesPerSec,
                                                                   double robotVelocityYInchesPerSec) {

        double distanceInches = distanceToGoalInches(robotX, robotY, robotHeadingRad);


        // --- Step A: stationary ballistic solution ---
        double hoodAngleDeg = hoodAngleCalculator(distanceInches);
        double v0 = velocityCalculator(distanceInches, hoodAngleDeg);


        double hoodAngleRad = Math.toRadians(hoodAngleDeg);

        // --- Step B1: decompose robot velocity into radial and tangential ---
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);
        double dx = goalXInches - releaseX;
        double dy = goalYInches - releaseY;
        double unitX = dx / distanceInches;
        double unitY = dy / distanceInches;

        // Radial: positive = robot moving toward goal
        double Vrr = robotVelocityXInchesPerSec * unitX + robotVelocityYInchesPerSec * unitY;
        // Tangential: positive = robot moving left of goal line
        double Vrt = robotVelocityXInchesPerSec * (-unitY) + robotVelocityYInchesPerSec * unitX;

        // --- Step B2: flight time from stationary solution ---
        double Vx0 = v0 * Math.cos(hoodAngleRad);
        double flightTime = distanceInches / Vx0;

        // --- Step B3: new horizontal ball speed ---
        // Ball must still cross distanceInches in flightTime,
        // but the robot is already contributing Vrr radially
        double Vx_compensated = (distanceInches / flightTime) - Vrr; // = Vx0 - Vrr



        // Add tangential demand as perpendicular horizontal component
        double Vx_new = Math.sqrt(Vx_compensated * Vx_compensated + Vrt * Vrt);

        // --- Step B4: vertical component unchanged ---
        double Vy = v0 * Math.sin(hoodAngleRad);

        // --- Step B5: new hood angle ---
        double newHoodAngleDeg = clamp(
                Math.toDegrees(Math.atan2(Vy, Vx_new)),
                CalculatorConstants.minHoodAngleDeg,
                CalculatorConstants.maxHoodAngleDeg
        );

        // --- Step B6: new launch speed using effective horizontal distance ---
        double x_new = Vx_new * flightTime;
        double v0_new = velocityCalculator(x_new, newHoodAngleDeg);


        // --- Step B7: turret heading offset ---
        double headingLeadRad = Math.atan2(Vrt, Vx_compensated);
        double targetHeadingRad = Math.atan2(dy, dx);
        double compensatedHeadingErrorRad = normalizeAngle(
                (targetHeadingRad - headingLeadRad) - robotHeadingRad
        );

        double rpm = rpmCalculator(v0_new);

        return new ShooterCalculatorRed.ShotSolution(
                distanceInches,
                compensatedHeadingErrorRad,
                baseHeadingErrorRad,
                headingLeadRad,
                newHoodAngleDeg,
                v0_new,
                rpm,
                Vrr,
                Vrt,
                false,
                null
        );
    }


}
