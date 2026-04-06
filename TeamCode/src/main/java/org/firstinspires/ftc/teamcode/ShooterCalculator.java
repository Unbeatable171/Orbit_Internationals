package org.firstinspires.ftc.teamcode;

public class ShooterCalculator {

    public enum GoalTarget {
        BLUE,
        RED
    }

    public enum FallbackZone {
        CLOSE,
        FAR
    }

    public static class ShotSolution {
        public final double distanceMeters;
        public final double headingErrorRad;
        public final double baseHeadingErrorRad;
        public final double headingLeadRad;
        public final double hoodAngleDeg;
        public final double velocityMetersPerSecond;
        public final double rpm;
        public final double towardTargetVelocityMetersPerSecond;
        public final double lateralVelocityMetersPerSecond;
        public final boolean usedFallback;
        public final FallbackZone fallbackZone;

        public ShotSolution(double distanceMeters,
                            double headingErrorRad,
                            double baseHeadingErrorRad,
                            double headingLeadRad,
                            double hoodAngleDeg,
                            double velocityMetersPerSecond,
                            double rpm,
                            double towardTargetVelocityMetersPerSecond,
                            double lateralVelocityMetersPerSecond,
                            boolean usedFallback,
                            FallbackZone fallbackZone) {
            this.distanceMeters = distanceMeters;
            this.headingErrorRad = headingErrorRad;
            this.baseHeadingErrorRad = baseHeadingErrorRad;
            this.headingLeadRad = headingLeadRad;
            this.hoodAngleDeg = hoodAngleDeg;
            this.velocityMetersPerSecond = velocityMetersPerSecond;
            this.rpm = rpm;
            this.towardTargetVelocityMetersPerSecond = towardTargetVelocityMetersPerSecond;
            this.lateralVelocityMetersPerSecond = lateralVelocityMetersPerSecond;
            this.usedFallback = usedFallback;
            this.fallbackZone = fallbackZone;
        }
    }

    private final double goalXMeters;
    private final double goalYMeters;
    private final GoalTarget goalTarget;

    public ShooterCalculator() {

        this(GoalTarget.BLUE);
    }

    public ShooterCalculator(GoalTarget goalTarget) {
        this.goalTarget = goalTarget;
        if (goalTarget == GoalTarget.RED) {
            goalXMeters = CalculatorConstants.redGoalXMeters;
            goalYMeters = CalculatorConstants.redGoalYMeters;
        } else {
            goalXMeters = CalculatorConstants.blueGoalXMeters;
            goalYMeters = CalculatorConstants.blueGoalYMeters;
        }
    }

    public double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    // ---------------- Geometry ----------------

    public double releaseXMeters(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotX
                + CalculatorConstants.releaseOffsetXMeters * cos
                - CalculatorConstants.releaseOffsetYMeters * sin;
    }

    public double releaseYMeters(double robotX, double robotY, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        return robotY
                + CalculatorConstants.releaseOffsetXMeters * sin
                + CalculatorConstants.releaseOffsetYMeters * cos;
    }

    public double distanceToGoalMeters(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXMeters(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYMeters(robotX, robotY, robotHeadingRad);

        double distanceX = goalXMeters - releaseX;
        double distanceY = goalYMeters - releaseY;

        return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    }

    public double headingChangeToGoal(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXMeters(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYMeters(robotX, robotY, robotHeadingRad);

        double distanceX = goalXMeters - releaseX;
        double distanceY = goalYMeters - releaseY;

        double targetHeading = Math.atan2(distanceY, distanceX);
        double headingError = targetHeading - robotHeadingRad;

        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        return headingError;
    }

    public double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }

    public double headingTurnPower(double headingErrorRad) {
        double headingErrorDeg = Math.abs(Math.toDegrees(headingErrorRad));
        double headingKp = headingErrorDeg <= CalculatorConstants.headingSecondaryThresholdDeg
                ? CalculatorConstants.headingSecondaryKp
                : CalculatorConstants.headingPrimaryKp;
        return clamp(
                headingErrorRad * headingKp,
                -CalculatorConstants.maxTurnPower,
                CalculatorConstants.maxTurnPower
        );
    }

    // ---------------- Calculators ----------------

    public double hoodAngleCalculator(double distanceMeters) {
        double hoodAngleDeg =
                CalculatorConstants.angleQuarticA * Math.pow(distanceMeters, 4)
                        + CalculatorConstants.angleQuarticB * Math.pow(distanceMeters, 3)
                        + CalculatorConstants.angleQuarticC * distanceMeters * distanceMeters
                        + CalculatorConstants.angleQuarticD * distanceMeters
                        + CalculatorConstants.angleQuarticE;

        /*
        double hoodAngleDeg = interpolateLookupTable(
                distanceMeters,
                CalculatorConstants.hoodLookupDistancesMeters,
                CalculatorConstants.hoodLookupAnglesDeg
        );
        */

        return clamp(
                hoodAngleDeg,
                CalculatorConstants.minHoodAngleDeg,
                CalculatorConstants.maxHoodAngleDeg
        );
    }

    /*
    private double interpolateLookupTable(double distanceMeters,
                                          double[] distancesMeters,
                                          double[] anglesDeg) {
        if (distancesMeters == null || anglesDeg == null
                || distancesMeters.length == 0
                || distancesMeters.length != anglesDeg.length) {
            return CalculatorConstants.defaultHoodAngleDeg;
        }

        if (distanceMeters <= distancesMeters[0]) {
            return anglesDeg[0];
        }

        for (int i = 1; i < distancesMeters.length; i++) {
            if (distanceMeters <= distancesMeters[i]) {
                double lowerDistance = distancesMeters[i - 1];
                double upperDistance = distancesMeters[i];
                double lowerAngle = anglesDeg[i - 1];
                double upperAngle = anglesDeg[i];
                double t = (distanceMeters - lowerDistance) / (upperDistance - lowerDistance);
                return lowerAngle + t * (upperAngle - lowerAngle);
            }
        }

        return anglesDeg[anglesDeg.length - 1];
    }
    */

    public double velocityCalculator(double distanceMeters, double hoodAngleDeg) {
        double x = distanceMeters;
        double y = CalculatorConstants.goalHeightDiffMeters;
        double angleRad = Math.toRadians(hoodAngleDeg);

        double cos = Math.cos(angleRad);
        double tan = Math.tan(angleRad);
        double denominator = 2 * cos * cos * (x * tan - y);

        if (denominator <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(
                (CalculatorConstants.gravityMetersPerSecSquared * x * x) / denominator
        );
    }

    public double rpmCalculator(double velocityMetersPerSecond) {
        double rpm =
                CalculatorConstants.rpmA * velocityMetersPerSecond * velocityMetersPerSecond
                        + CalculatorConstants.rpmB * velocityMetersPerSecond
                        + CalculatorConstants.rpmC;

        return clamp(rpm, CalculatorConstants.minRpm, CalculatorConstants.maxRpm);
    }

    // ---------------- Fallback ----------------

    private double mirrorXInches(double xInches) {
        return 144.0 - xInches;
    }

    private double triangleX(double xInches) {
        return goalTarget == GoalTarget.RED ? mirrorXInches(xInches) : xInches;
    }

    private boolean isInsideTriangleInches(double pointXInches, double pointYInches,
                                           double ax, double ay,
                                           double bx, double by,
                                           double cx, double cy) {
        double sideAB = (pointXInches - ax) * (by - ay) - (pointYInches - ay) * (bx - ax);
        double sideBC = (pointXInches - bx) * (cy - by) - (pointYInches - by) * (cx - bx);
        double sideCA = (pointXInches - cx) * (ay - cy) - (pointYInches - cy) * (ax - cx);
        boolean hasNegative = sideAB < 0 || sideBC < 0 || sideCA < 0;
        boolean hasPositive = sideAB > 0 || sideBC > 0 || sideCA > 0;
        return !(hasNegative && hasPositive);
    }

    public FallbackZone getFallbackZone(double robotX, double robotY) {
        double robotXInches = robotX / 0.0254;
        double robotYInches = robotY / 0.0254;

        if (isInsideTriangleInches(
                robotXInches, robotYInches,
                triangleX(CalculatorConstants.closeFallbackTriangleAXInches), CalculatorConstants.closeFallbackTriangleAYInches,
                triangleX(CalculatorConstants.closeFallbackTriangleBXInches), CalculatorConstants.closeFallbackTriangleBYInches,
                triangleX(CalculatorConstants.closeFallbackTriangleCXInches), CalculatorConstants.closeFallbackTriangleCYInches)) {
            return FallbackZone.CLOSE;
        }

        return FallbackZone.FAR;
    }

    public double fallbackHoodAngle(FallbackZone zone) {
        switch (zone) {
            case CLOSE:
                return CalculatorConstants.closeFallbackHoodAngleDeg;
            default:
                return CalculatorConstants.farFallbackHoodAngleDeg;
        }
    }

    public double fallbackRpm(FallbackZone zone) {
        switch (zone) {
            case CLOSE:
                return CalculatorConstants.closeFallbackRpm;
            default:
                return CalculatorConstants.farFallbackRpm;
        }
    }

    // ---------------- Single Decision Path ----------------

    public ShotSolution calculateShotSolution(double robotX, double robotY, double robotHeadingRad) {
        return calculateShotSolution(robotX, robotY, robotHeadingRad, 0.0, 0.0);
    }

    public ShotSolution calculateShotSolution(double robotX,
                                              double robotY,
                                              double robotHeadingRad,
                                              double robotVelocityXMetersPerSecond,
                                              double robotVelocityYMetersPerSecond) {
        double distanceMeters = distanceToGoalMeters(robotX, robotY, robotHeadingRad);
        double baseHeadingErrorRad = headingChangeToGoal(robotX, robotY, robotHeadingRad);

        if (distanceMeters <= 1e-6) {
            FallbackZone fallbackZone = getFallbackZone(robotX, robotY);

            return new ShotSolution(
                    distanceMeters,
                    baseHeadingErrorRad,
                    baseHeadingErrorRad,
                    0.0,
                    fallbackHoodAngle(fallbackZone),
                    Double.NaN,
                    fallbackRpm(fallbackZone),
                    0.0,
                    0.0,
                    true,
                    fallbackZone
            );
        }

        double releaseX = releaseXMeters(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYMeters(robotX, robotY, robotHeadingRad);
        double goalDeltaX = goalXMeters - releaseX;
        double goalDeltaY = goalYMeters - releaseY;

        double targetHeadingRad = Math.atan2(goalDeltaY, goalDeltaX);
        double unitX = goalDeltaX / distanceMeters;
        double unitY = goalDeltaY / distanceMeters;

        double towardTargetVelocityMetersPerSecond =
                robotVelocityXMetersPerSecond * unitX + robotVelocityYMetersPerSecond * unitY;
        double lateralVelocityMetersPerSecond =
                robotVelocityXMetersPerSecond * (-unitY) + robotVelocityYMetersPerSecond * unitX;

        double calculatedHoodAngleDeg = hoodAngleCalculator(distanceMeters);
        double calculatedVelocityMetersPerSecond = velocityCalculator(distanceMeters, calculatedHoodAngleDeg);

        if (Double.isNaN(calculatedVelocityMetersPerSecond)) {
            FallbackZone fallbackZone = getFallbackZone(robotX, robotY);
            double fallbackHoodAngleDeg = fallbackHoodAngle(fallbackZone);
            double fallbackRpm = fallbackRpm(fallbackZone);

            return new ShotSolution(
                    distanceMeters,
                    baseHeadingErrorRad,
                    baseHeadingErrorRad,
                    0.0,
                    fallbackHoodAngleDeg,
                    Double.NaN,
                    fallbackRpm,
                    towardTargetVelocityMetersPerSecond,
                    lateralVelocityMetersPerSecond,
                    true,
                    fallbackZone
            );
        }

        double flightTimeSeconds = distanceMeters / Math.max(calculatedVelocityMetersPerSecond, 1e-6);
        double headingLeadRad = Math.atan2(lateralVelocityMetersPerSecond * flightTimeSeconds, distanceMeters);
        double compensatedTargetHeadingRad = targetHeadingRad - headingLeadRad;
        double compensatedHeadingErrorRad = normalizeAngle(compensatedTargetHeadingRad - robotHeadingRad);

        double compensatedVelocityMetersPerSecond = calculatedVelocityMetersPerSecond - towardTargetVelocityMetersPerSecond;
        if (compensatedVelocityMetersPerSecond <= 0) {
            FallbackZone fallbackZone = getFallbackZone(robotX, robotY);
            double fallbackHoodAngleDeg = fallbackHoodAngle(fallbackZone);
            double fallbackRpm = fallbackRpm(fallbackZone);

            return new ShotSolution(
                    distanceMeters,
                    compensatedHeadingErrorRad,
                    baseHeadingErrorRad,
                    headingLeadRad,
                    fallbackHoodAngleDeg,
                    Double.NaN,
                    fallbackRpm,
                    towardTargetVelocityMetersPerSecond,
                    lateralVelocityMetersPerSecond,
                    true,
                    fallbackZone
            );
        }

        double calculatedRpm = rpmCalculator(compensatedVelocityMetersPerSecond);

        return new ShotSolution(
                distanceMeters,
                compensatedHeadingErrorRad,
                baseHeadingErrorRad,
                headingLeadRad,
                calculatedHoodAngleDeg,
                compensatedVelocityMetersPerSecond,
                calculatedRpm,
                towardTargetVelocityMetersPerSecond,
                lateralVelocityMetersPerSecond,
                false,
                null
        );
    }
}
