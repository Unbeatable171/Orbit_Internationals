package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;

@Configurable
public class Turrettt extends SubsystemBase {

    public final ServoEx servoRightBack;
    public final ServoEx servoLeftBack;
    public final ServoExGroup turretServos;

    /*
     * Logical turret-angle range:
     *
     * 0 radians  = 0 degrees
     * 2π radians = 360 degrees
     */
    public static double minimumValueRad = 0;
    public static double maximumValueRad = 2*Math.PI;

    /*
     * Logical ServoEx range for one full turret rotation.
     */
    public static double minPosServos = 0.1375;
    public static double maxPosServos = 0.82;

    /*
     * Maximum logical servo-position change per periodic loop.
     */
    public static double maxStepPerLoop = 0.04                                                                                                                                                                                                                                                                                                          ;

    /*
     * Wrap detection regions.
     */
    public static double wrapLow = minPosServos + 0.02;
    public static double wrapHigh = maxPosServos - 0.07;

    /*
     * This is a physically safe unwind location.
     *
     * It is not the mathematical midpoint of 0.08–0.895.
     */
    public static double safeMiddle = 0.40;

    /*
     * The turret is mounted opposite Pedro Pathing's
     * robot-forward direction.
     */
    public static double turretMountOffset = Math.PI;

    private double currentServoPosition = safeMiddle;

    private double finalTargetPosition = safeMiddle;

    private double previousTargetPosition = safeMiddle;

    private boolean routingThroughMiddle = false;

    private double currentMovementLeadRadians = 0.0;
    private double currentTargetHeadingRadians = 0.0;

    public Turrettt(HardwareMap hardwareMap) {

        servoRightBack = new ServoEx(
                hardwareMap,
                "turretservoright"
        );

        servoLeftBack = new ServoEx(
                hardwareMap,
                "turretservoleft"
        );

        PwmControl.PwmRange turretPwmRange =
                new PwmControl.PwmRange(500, 2500);

        servoRightBack.setPwm(turretPwmRange);
        servoLeftBack.setPwm(turretPwmRange);

        turretServos = new ServoExGroup(
                servoRightBack,
                servoLeftBack
        );

        servoLeftBack.setInverted(false);
        servoRightBack.setInverted(false);

        commandPosition(currentServoPosition);
    }

    @Override
    public void periodic() {

        Pose robotPose = Localization.getPose();

        Pose goalPose = getGoalPose();

        currentTargetHeadingRadians =
                calculateTargetHeading(
                        robotPose,
                        goalPose
                );

        double targetServoPosition =
                headingToTurretPos(
                        currentTargetHeadingRadians
                );

        moveToPosition(targetServoPosition);
    }

    private Pose getGoalPose() {

        return "RED".equalsIgnoreCase(
                RobotConstants.chosenAlliance
        )
                ? RobotConstants.redGoalPose
                : RobotConstants.blueGoalPose;
    }

    /**
     * Calculates turret angle relative to the robot and
     * includes shooting-while-moving lead.
     */
    public double calculateTargetHeading(
            Pose robotPose,
            Pose goalPose
    ) {

        double dx =
                goalPose.getX()
                        - robotPose.getX();

        double dy =
                goalPose.getY()
                        - robotPose.getY();

        double absoluteTargetHeading =
                Math.atan2(dy, dx);

        currentMovementLeadRadians = 0;

        /*
         * Add the lead to the field-relative goal heading.
         */
        absoluteTargetHeading += currentMovementLeadRadians;

        double robotHeading = robotPose.getHeading();

        double turretHeading =
                absoluteTargetHeading
                        - robotHeading
                        + turretMountOffset;

        return normalizeRadians(turretHeading);
    }

    /**
     * Converts:
     *
     * 0–2π radians
     *
     * into:
     *
     * minPosServos–maxPosServos
     */
    public double headingToTurretPos(
            double headingRadians
    ) {

        double position =
                (
                        (maxPosServos - minPosServos)
                                * (
                                headingRadians
                                        - minimumValueRad
                        )
                                / (
                                maximumValueRad
                                        - minimumValueRad
                        )
                ) + minPosServos;

        return clamp(position);
    }

    /**
     * Converts an angle to:
     *
     * 0 <= angle < 2π
     */
    private double normalizeRadians(double angle) {

        double fullRotation = 2.0 * Math.PI;

        double normalized = angle % fullRotation;

        if (normalized < 0.0) {
            normalized += fullRotation;
        }

        return normalized;
    }

    private double clamp(double position) {

        return Math.max(
                minPosServos,
                Math.min(
                        maxPosServos,
                        position
                )
        );
    }

    /**
     * Handles direct aiming and 0°/360° wrap crossings.
     */
    private void moveToPosition(
            double targetPosition
    ) {

        targetPosition = clamp(targetPosition);

        finalTargetPosition = targetPosition;

        boolean crossHighToLow =
                previousTargetPosition > wrapHigh
                        && targetPosition < wrapLow;

        boolean crossLowToHigh =
                previousTargetPosition < wrapLow
                        && targetPosition > wrapHigh;

        if (!routingThroughMiddle
                && (
                crossHighToLow
                        || crossLowToHigh
        )) {

            routingThroughMiddle = true;
        }

        previousTargetPosition = targetPosition;

        double activeTarget =
                routingThroughMiddle
                        ? clamp(safeMiddle)
                        : finalTargetPosition;

        double nextPosition =
                moveToward(
                        currentServoPosition,
                        activeTarget,
                        maxStepPerLoop
                );

        commandPosition(nextPosition);

        if (routingThroughMiddle
                && Math.abs(
                currentServoPosition
                        - clamp(safeMiddle)
        ) < 0.0001) {

            routingThroughMiddle = false;
        }
    }

    /**
     * Smoothly moves toward the requested servo position.
     */
    private double moveToward(
            double current,
            double target,
            double maxStep
    ) {

        double error = target - current;

        if (Math.abs(error) <= maxStep) {
            return target;
        }

        double dynamicStep =
                Math.min(
                        Math.abs(error) * 0.4,
                        maxStep
                );

        return current
                + Math.signum(error)
                * dynamicStep;
    }

    /**
     * Sends the logical position to both turret servos.
     */
    public void commandPosition(
            double position
    ) {

        position = clamp(position);

        currentServoPosition = position;

        turretServos.set(position);
    }

    public void setPosition(double position) {
        moveToPosition(position);
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    public double getFinalTargetPosition() {
        return finalTargetPosition;
    }

    public boolean isRoutingThroughMiddle() {
        return routingThroughMiddle;
    }

    public double getCurrentTargetHeadingRadians() {
        return currentTargetHeadingRadians;
    }

    public double getCurrentTargetHeadingDegrees() {

        return Math.toDegrees(
                currentTargetHeadingRadians
        );
    }

    public double getMovementLeadRadians() {
        return currentMovementLeadRadians;
    }

    public double getMovementLeadDegrees() {

        return Math.toDegrees(
                currentMovementLeadRadians
        );
    }

    public double getTargetHeadingDeg(
            Pose robotPose,
            Pose goalPose
    ) {

        return Math.toDegrees(
                calculateTargetHeading(
                        robotPose,
                        goalPose
                )
        );
    }
}