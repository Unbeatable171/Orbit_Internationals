package org.firstinspires.ftc.teamcode;

public class ShooterCalculator {


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
                + CONSTANTS.releaseOffsetXInches * cos
                - CONSTANTS.releaseOffsetYInches * sin;
    }

    public double releaseYInches(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotY
                + CONSTANTS.releaseOffsetXInches * sin
                + CONSTANTS.releaseOffsetYInches * cos;
    }

    public double distanceToGoalInches(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);

        double distanceX = CONSTANTS.blueGoalXInches - releaseX;
        double distanceY = CONSTANTS.blueGoalYInches - releaseY;

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
//mmm
        if(denominator <= 0){
            return Double.NaN;
        }

        return Math.sqrt(
                (CONSTANTS.gravityInches * x * x) / denominator
        );
    }

    public double rpmCalculator(double velocityInchesPerSec) {
        double rpm = CONSTANTS.rpmA * velocityInchesPerSec * velocityInchesPerSec
                + CONSTANTS.rpmB * velocityInchesPerSec
                + CONSTANTS.rpmC - 250;

            return clamp(rpm, CONSTANTS.minRpm, CONSTANTS.maxRpm);
    }

    public ShooterCalculator.ShotSolution calculateShotSolution(double robotX, double robotY, double robotHeadingRad) {
        return calculateShotSolution(robotX, robotY, robotHeadingRad, 0.0, 0.0);
    }

    public ShooterCalculator.ShotSolution calculateShotSolution(double robotX, double robotY,
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
        double dx = CONSTANTS.blueGoalXInches - releaseX;
        double dy = CONSTANTS.blueGoalYInches - releaseY;
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
                CONSTANTS.minHoodAngleDeg,
                CONSTANTS.maxHoodAngleDeg
        );

        // --- Step B6: new launch speed using effective horizontal distance ---
        double x_new = Vx_new * flightTime;
        double v0_new = velocityCalculator(x_new, newHoodAngleDeg);
//mmm

        if(Double.isNaN((v0_new))||v0_new <=0){
            v0_new = v0;
            newHoodAngleDeg = hoodAngleDeg;
        }

        // --- Step B7: turret heading offset ---
        double headingLeadRad = CONSTANTS.turretLeadScale * Math.atan2(Vrt, Vx_compensated);
        double targetHeadingRad = Math.atan2(dy, dx);
        double compensatedTargetHeadingRad = normalizeAngle(targetHeadingRad - headingLeadRad);
        double compensatedHeadingErrorRad = normalizeAngle(
                compensatedTargetHeadingRad - robotHeadingRad
        );

        double rpm = rpmCalculator(v0_new);

        return new ShooterCalculator.ShotSolution(
                distanceInches,
                newHoodAngleDeg,
                v0_new,
                rpm,
                Vrr,
                Vrt,
                compensatedTargetHeadingRad,
                compensatedHeadingErrorRad,
                compensatedHeadingErrorRad
        );
    }
}
