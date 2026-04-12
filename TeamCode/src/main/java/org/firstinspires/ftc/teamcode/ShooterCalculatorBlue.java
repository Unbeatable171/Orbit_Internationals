package org.firstinspires.ftc.teamcode;

public class ShooterCalculatorBlue {

    public double velocityoffset = 50;

    public enum GoalTarget {
        BLUE,
        RED
    }

    public enum FallbackZone {
        CLOSE,
        FAR
    }

    public static class ShotSolution {

        public final double distanceInches;
        public final double headingErrorRad;
        public final double baseHeadingErrorRad;
        public final double headingLeadRad;
        public final double hoodAngleDeg;
        public final double launchVelocityInchesPerSec;
        public final double rpm;
        public final double radialVelocityInchesPerSec;
        public final double tangentialVelocityInchesPerSec;
        public final boolean usedFallback;
        public final FallbackZone fallbackZone;

        public ShotSolution(double distanceInches,
                            double headingErrorRad,
                            double baseHeadingErrorRad,
                            double headingLeadRad,
                            double hoodAngleDeg,
                            double launchVelocityInchesPerSec,
                            double rpm,
                            double radialVelocityInchesPerSec,
                            double tangentialVelocityInchesPerSec,
                            boolean usedFallback,
                            FallbackZone fallbackZone) {
            this.distanceInches = distanceInches;
            this.headingErrorRad = headingErrorRad;
            this.baseHeadingErrorRad = baseHeadingErrorRad;
            this.headingLeadRad = headingLeadRad;
            this.hoodAngleDeg = hoodAngleDeg;
            this.launchVelocityInchesPerSec = launchVelocityInchesPerSec;
            this.rpm = rpm;
            this.radialVelocityInchesPerSec = radialVelocityInchesPerSec;
            this.tangentialVelocityInchesPerSec = tangentialVelocityInchesPerSec;
            this.usedFallback = usedFallback;
            this.fallbackZone = fallbackZone;
        }
    }

    private final double goalXInches;
    private final double goalYInches;
    private final GoalTarget goalTarget;

    private double lastHeadingErrorRad = 0;
    private long lastHeadingTimestamp  = System.currentTimeMillis();

    private final double gravity = CalculatorConstants.gravityInches;
    private final double goalHeight = CalculatorConstants.goalHeightInches;
    private final double scoreAngle = CalculatorConstants.scoreAngle;

    public ShooterCalculatorBlue() {
        this(GoalTarget.BLUE);
    }

    public ShooterCalculatorBlue(GoalTarget goalTarget) {
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
        double dx = goalXInches - releaseX;
        double dy = goalYInches - releaseY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double headingChangeToGoal(double robotX, double robotY, double robotHeadingRad) {
        double releaseX = releaseXInches(robotX, robotY, robotHeadingRad);
        double releaseY = releaseYInches(robotX, robotY, robotHeadingRad);
        double dx = goalXInches - releaseX;
        double dy = goalYInches - releaseY;
        double targetHeading = (240 + Math.atan((144-dy)/(dx)));
        double headingError = targetHeading - robotHeadingRad;
        while (headingError > Math.PI)  headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        return headingError;
    }

    public double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI)  angleRad -= 2 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }

    public double headingTurnPower(double headingErrorRad) {
        double headingErrorDeg = Math.abs(Math.toDegrees(headingErrorRad));

        boolean useSecondary = headingErrorDeg <= CalculatorConstants.headingSecondaryThresholdDeg;

        double kp = useSecondary
                ? CalculatorConstants.headingSecondaryKp
                : CalculatorConstants.headingPrimaryKp;
        double kd = useSecondary
                ? CalculatorConstants.headingSecondaryKd
                : CalculatorConstants.headingPrimaryKd;

        long now = System.currentTimeMillis();
        double dt = (now - lastHeadingTimestamp) / 1000.0;
        double derivative = (dt > 0) ? (headingErrorRad - lastHeadingErrorRad) / dt : 0;

        lastHeadingErrorRad  = headingErrorRad;
        lastHeadingTimestamp = now;

        return clamp((headingErrorRad * kp) + (derivative * kd),
                -CalculatorConstants.maxTurnPower,
                CalculatorConstants.maxTurnPower);
    }
    public void resetHeadingPD() {
        lastHeadingErrorRad  = 0;
        lastHeadingTimestamp = System.currentTimeMillis();
    }
    // ---------------- Ballistic Calculators ----------------

    public double hoodAngleCalculator(double distanceInches) {
        double hoodAngleRad = Math.atan(
                (2.0 * goalHeight / distanceInches) - Math.tan(Math.toRadians(scoreAngle))
        );
        return clamp(
                Math.toDegrees(hoodAngleRad),
                CalculatorConstants.minHoodAngleDeg,
                CalculatorConstants.maxHoodAngleDeg
        );
    }

    public double velocityCalculator(double distanceInches, double hoodAngleDeg) {
        double angleRad = Math.toRadians(hoodAngleDeg);
        double cos = Math.cos(angleRad);
        double tan = Math.tan(angleRad);
        double denominator = 2.0 * cos * cos * (distanceInches * tan - goalHeight);
        if (denominator <= 0) return Double.NaN;
        return Math.sqrt((gravity * distanceInches * distanceInches) / denominator)+velocityoffset;
    }

    public double rpmCalculator(double velocityInchesPerSec) {
        double rpm = CalculatorConstants.rpmA * velocityInchesPerSec * velocityInchesPerSec * velocityInchesPerSec
                + CalculatorConstants.rpmB * velocityInchesPerSec * velocityInchesPerSec
                + CalculatorConstants.rpmC* velocityInchesPerSec
                + CalculatorConstants.rpmD;
        return clamp(rpm, CalculatorConstants.minRpm, CalculatorConstants.maxRpm);
    }

    // ---------------- Fallback ----------------

    private double mirrorXInches(double xInches) {
        return 144 - xInches;
    }

    private double triangleX(double xInches) {
        return goalTarget == GoalTarget.RED ? mirrorXInches(xInches) : xInches;
    }

    private boolean isInsideTriangleInches(double px, double py,
                                           double ax, double ay,
                                           double bx, double by,
                                           double cx, double cy) {
        double sideAB = (px - ax) * (by - ay) - (py - ay) * (bx - ax);
        double sideBC = (px - bx) * (cy - by) - (py - by) * (cx - bx);
        double sideCA = (px - cx) * (ay - cy) - (py - cy) * (ax - cx);
        boolean hasNeg = sideAB < 0 || sideBC < 0 || sideCA < 0;
        boolean hasPos = sideAB > 0 || sideBC > 0 || sideCA > 0;
        return !(hasNeg && hasPos);
    }

    public FallbackZone getFallbackZone(double robotX, double robotY) {
        if (isInsideTriangleInches(
                robotX, robotY,
                triangleX(CalculatorConstants.closeFallbackTriangleAXInches), CalculatorConstants.closeFallbackTriangleAYInches,
                triangleX(CalculatorConstants.closeFallbackTriangleBXInches), CalculatorConstants.closeFallbackTriangleBYInches,
                triangleX(CalculatorConstants.closeFallbackTriangleCXInches), CalculatorConstants.closeFallbackTriangleCYInches)) {
            return FallbackZone.CLOSE;
        }
        return FallbackZone.FAR;
    }

    public double fallbackHoodAngle(FallbackZone zone) {
        return zone == FallbackZone.CLOSE
                ? CalculatorConstants.closeFallbackHoodAngleDeg
                : CalculatorConstants.farFallbackHoodAngleDeg;
    }

    public double fallbackRpm(FallbackZone zone) {
        return zone == FallbackZone.CLOSE
                ? CalculatorConstants.closeFallbackRpm
                : CalculatorConstants.farFallbackRpm;
    }

    private ShotSolution makeFallback(double distanceInches,
                                      double headingErrorRad,
                                      double baseHeadingErrorRad,
                                      double headingLeadRad,
                                      double robotX, double robotY,
                                      double Vrr, double Vrt) {
        FallbackZone zone = getFallbackZone(robotX, robotY);
        return new ShotSolution(
                distanceInches,
                headingErrorRad,
                baseHeadingErrorRad,
                headingLeadRad,
                fallbackHoodAngle(zone),
                Double.NaN,
                fallbackRpm(zone),
                Vrr,
                Vrt,
                true,
                zone
        );
    }

    // ---------------- Shot Solution ----------------

    public ShotSolution calculateShotSolution(double robotX, double robotY, double robotHeadingRad) {
        return calculateShotSolution(robotX, robotY, robotHeadingRad, 0.0, 0.0);
    }

    /**
     * @param robotX                  release point X in inches (Pedro Pathing frame)
     * @param robotY                  release point Y in inches
     * @param robotHeadingRad         robot heading in radians
     * @param robotVelocityXInchesPerSec  field-frame X velocity in inches/sec
     * @param robotVelocityYInchesPerSec  field-frame Y velocity in inches/sec
     */
    public ShotSolution calculateShotSolution(double robotX, double robotY,
                                              double robotHeadingRad,
                                              double robotVelocityXInchesPerSec,
                                              double robotVelocityYInchesPerSec) {

        double distanceInches = distanceToGoalInches(robotX, robotY, robotHeadingRad);
        double baseHeadingErrorRad = headingChangeToGoal(robotX, robotY, robotHeadingRad);

        // --- Degenerate distance fallback ---
        if (distanceInches <= 1e-6) {
            return makeFallback(distanceInches, baseHeadingErrorRad, baseHeadingErrorRad,
                    0.0, robotX, robotY, 0.0, 0.0);
        }

        // --- Step A: stationary ballistic solution ---
        double hoodAngleDeg = hoodAngleCalculator(distanceInches);
        double v0 = velocityCalculator(distanceInches, hoodAngleDeg);

        if (Double.isNaN(v0)) {
            return makeFallback(distanceInches, baseHeadingErrorRad, baseHeadingErrorRad,
                    0.0, robotX, robotY, 0.0, 0.0);
        }

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

        if (Vx_compensated <= 0) {
            // Robot is moving toward goal faster than ball needs to — fallback
            return makeFallback(distanceInches, baseHeadingErrorRad, baseHeadingErrorRad,
                    0.0, robotX, robotY, Vrr, Vrt);
        }

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

        if (Double.isNaN(v0_new)) {
            return makeFallback(distanceInches, baseHeadingErrorRad, baseHeadingErrorRad,
                    0.0, robotX, robotY, Vrr, Vrt);
        }

        // --- Step B7: turret heading offset ---
        double headingLeadRad = Math.atan2(Vrt, Vx_compensated);
        double targetHeadingRad = Math.atan2(dy, dx);
        double compensatedHeadingErrorRad = normalizeAngle(
                (targetHeadingRad - headingLeadRad) - robotHeadingRad
        );

        double rpm = rpmCalculator(v0_new);

        return new ShotSolution(
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
