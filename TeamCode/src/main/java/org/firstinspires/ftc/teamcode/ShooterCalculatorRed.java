package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

@Config
public class ShooterCalculatorRed {

    public static volatile double releaseOffsetXInches = 0.0;
    public static volatile double releaseOffsetYInches = -1.5;
    public static volatile double rpmD = 140;
    public static volatile double rpmE = 120;

    public static class ShotSolution {
        public final double distanceInches;
        public final double hoodAngleDeg;
        public final double velocityInchesPerSecond;
        public final double rpm;
        public final double towardTargetVelocityInchesPerSecond;
        public final double lateralVelocityInchesPerSecond;
        public final double targetHeadingRad;
        public final double headingErrorRad;
        public final double turretAngleRad;

        public ShotSolution(double distanceInches,
                            double hoodAngleDeg,
                            double velocityInchesPerSecond,
                            double rpm,
                            double towardTargetVelocityInchesPerSecond,
                            double lateralVelocityInchesPerSecond,
                            double targetHeadingRad,
                            double headingErrorRad,
                            double turretAngleRad) {
            this.distanceInches = distanceInches;
            this.hoodAngleDeg = hoodAngleDeg;
            this.velocityInchesPerSecond = velocityInchesPerSecond;
            this.rpm = rpm;
            this.towardTargetVelocityInchesPerSecond = towardTargetVelocityInchesPerSecond;
            this.lateralVelocityInchesPerSecond = lateralVelocityInchesPerSecond;
            this.targetHeadingRad = targetHeadingRad;
            this.headingErrorRad = headingErrorRad;
            this.turretAngleRad = turretAngleRad;
        }
    }

    public double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    public double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }

    public double releaseXInches(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotX
                + releaseOffsetXInches * cos
                - releaseOffsetYInches * sin;
    }

    public double releaseYInches(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotY
                + releaseOffsetXInches * sin
                + releaseOffsetYInches * cos;
    }

    public double distanceToGoalInches(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);

        double distanceX = RobotConstants.redGoalPose.getX() - releaseX;
        double distanceY = RobotConstants.redGoalPose.getY() - releaseY;

        return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    }

    public double hoodAngleCalculator(double distanceInches) {
        double hoodAngleDeg = Math.toDegrees(
                Math.atan((2 * CONSTANTS.goalHeightInches / distanceInches)
                        - Math.tan(Math.toRadians(CONSTANTS.scoreAngle)))
        );
        return clamp(
                hoodAngleDeg,
                CONSTANTS.minHoodAngleDeg,
                CONSTANTS.maxHoodAngleDeg
        );
    }

    public double velocityCalculator(double distanceInches, double hoodAngleDeg) {
        double x = distanceInches;
        double y = CONSTANTS.goalHeightInches;
        double angleRad = Math.toRadians(hoodAngleDeg);

        double cos = Math.cos(angleRad);
        double tan = Math.tan(angleRad);
        double denominator = 2 * cos * cos * (x * tan - y);

        if (denominator <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(
                (CONSTANTS.gravityInches * x * x) / denominator
        );
    }

    public double rpmCalculator(double velocityInchesPerSec) {
        double rpm = CONSTANTS.rpmA * velocityInchesPerSec * velocityInchesPerSec
                + CONSTANTS.rpmB * velocityInchesPerSec
                + CONSTANTS.rpmC;

        if (rpm <= 2900) {
            double closerpm = rpm - rpmD;
            return clamp(closerpm, CONSTANTS.minRpm, CONSTANTS.maxRpm);
        } else {
            double rpmFar = rpm - rpmE;
            return clamp(rpmFar, CONSTANTS.minRpm, CONSTANTS.maxRpm);
        }
    }

    public ShooterCalculatorRed.ShotSolution calculateShotSolution(double robotX, double robotY, double robotHeadingRad) {
        return calculateShotSolution(robotX, robotY, robotHeadingRad, 0.0, 0.0);
    }

    public ShooterCalculatorRed.ShotSolution calculateShotSolution(double robotX, double robotY,
                                                                   double robotHeadingRad,
                                                                   double robotVelocityXInchesPerSec,
                                                                   double robotVelocityYInchesPerSec) {
        double distanceInches = distanceToGoalInches(robotX, robotY, robotHeadingRad);

        double hoodAngleDeg = hoodAngleCalculator(distanceInches);
        double v0 = velocityCalculator(distanceInches, hoodAngleDeg);

        double hoodAngleRad = Math.toRadians(hoodAngleDeg);

        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);
        double dx = RobotConstants.redGoalPose.getX() - releaseX;
        double dy = RobotConstants.redGoalPose.getY() - releaseY;
        double unitX = dx / distanceInches;
        double unitY = dy / distanceInches;

        double Vrr = robotVelocityXInchesPerSec * unitX + robotVelocityYInchesPerSec * unitY;
        double Vrt = robotVelocityXInchesPerSec * (-unitY) + robotVelocityYInchesPerSec * unitX;

        double Vx0 = v0 * Math.cos(hoodAngleRad);
        double flightTime = distanceInches / Vx0;
        double VxCompensated = (distanceInches / flightTime) - Vrr;
        double VxNew = Math.sqrt(VxCompensated * VxCompensated + Vrt * Vrt);
        double Vy = v0 * Math.sin(hoodAngleRad);

        double newHoodAngleDeg = clamp(
                Math.toDegrees(Math.atan2(Vy, VxNew)),
                CONSTANTS.minHoodAngleDeg,
                CONSTANTS.maxHoodAngleDeg
        );

        double xNew = VxNew * flightTime;
        double v0New = velocityCalculator(xNew, newHoodAngleDeg);

        if (Double.isNaN(v0New) || v0New <= 0) {
            v0New = v0;
            newHoodAngleDeg = hoodAngleDeg;
        }

        double headingLeadRad = CONSTANTS.turretLeadScale * Math.atan2(Vrt, VxCompensated);
        double targetHeadingRad = Math.atan2(dy, dx);
        double compensatedTargetHeadingRad = normalizeAngle(targetHeadingRad - headingLeadRad);
        double compensatedHeadingErrorRad = normalizeAngle(
                compensatedTargetHeadingRad - robotHeadingRad
        );

        double rpm = rpmCalculator(v0New);

        return new ShooterCalculatorRed.ShotSolution(
                distanceInches,
                newHoodAngleDeg,
                v0New,
                rpm,
                Vrr,
                Vrt,
                compensatedTargetHeadingRad,
                compensatedHeadingErrorRad,
                compensatedHeadingErrorRad
        );
    }
}
