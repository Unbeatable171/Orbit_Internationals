package org.firstinspires.ftc.teamcode;

public class ShooterCalculator1 {

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

    private final double goalXInches;
    private final double goalYInches;
    private final GoalTarget goalTarget;

    private double g = CalculatorConstants.gravityInches;
    private double y = CalculatorConstants.goalHeightInches;
    private double a = CalculatorConstants.scoreAngle;

    public ShooterCalculator1() {

        this(GoalTarget.BLUE);
    }

    public ShooterCalculator1(GoalTarget goalTarget) {
        this.goalTarget = goalTarget;
        if (goalTarget == GoalTarget.RED) {
            goalXInches = CalculatorConstants.redGoalXInches;
            goalYInches = CalculatorConstants.redGoalYInches;
        } else {
            goalXInches = CalculatorConstants.blueGoalXInches;
            goalYInches = CalculatorConstants.blueGoalYInches;
        }
    }

    public double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    // ---------------- Geometry ----------------

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

    public double headingChangeToGoal(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);

        double distanceX = goalXInches - releaseX;
        double distanceY = goalYInches - releaseY;

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
        double hoodAngleDeg = Math.atan((2*y/distanceMeters) - Math.tan(a));
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

        if (denominator <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(
                (CalculatorConstants.gravityInches * x * x) / denominator
        );
    }

    public double rpmCalculator(double velocityMetersPerSecond) {
        double rpm =
                CalculatorConstants.rpmA * velocityMetersPerSecond * velocityMetersPerSecond
                        + CalculatorConstants.rpmB * velocityMetersPerSecond
                        + CalculatorConstants.rpmC;

        return clamp(rpm, CalculatorConstants.minRpm, CalculatorConstants.maxRpm);
    }


}
