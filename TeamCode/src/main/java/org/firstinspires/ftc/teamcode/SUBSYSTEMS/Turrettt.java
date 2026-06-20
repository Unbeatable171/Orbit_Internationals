// Turret.java
package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.globals.Localization;

@Configurable
public class Turrettt extends SubsystemBase {

    public final ServoEx servoRightBack;
    public final ServoEx servoLeftBack;
    public final ServoExGroup turretServos;

    public static double minimumValueRad = -Math.PI;
    public static double maximumValueRad = Math.PI;

    public static double minPosServos = 0.155;
    public static double maxPosServos = 0.855;

    public static double maxStepPerLoop = 0.03;

    public static double wrapLow = 0.165;
    public static double wrapHigh = 0.835;

    public static double safeMiddle = 0.335   ;

    public static double turretMountOffset = 0.0;


    private double currentServoPosition = safeMiddle;
    private double finalTargetPosition = safeMiddle;
    private double previousTargetPosition = safeMiddle;

    private boolean routingThroughMiddle = false;

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

        Pose goalPose = "RED".equals(chosenAlliance)
                ? redGoalPose
                : blueGoalPose;

        double targetHeading =
                calculateTargetHeading(robotPose, goalPose);

        double targetServoPosition =
                headingToTurretPos(targetHeading);

        moveToPosition(targetServoPosition);
    }

    public double calculateTargetHeading(
            Pose robotPose,
            Pose goalPose
    ) {

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();


        double absoluteTargetHeading = Math.atan2(dy, dx);

        double robotHeading = robotPose.getHeading();

        double turretHeading =
                absoluteTargetHeading
                        - robotHeading
                        + turretMountOffset;

        return normalizeSignedRadians(turretHeading);
    }


    public double headingToTurretPos(double headingRad) {

        double position =
                ((maxPosServos - minPosServos)
                        * (headingRad - minimumValueRad)
                        / (maximumValueRad - minimumValueRad))
                        + minPosServos;

        return clamp(position);
    }


    private double normalizeSignedRadians(double angle) {

        double fullRotation = 2.0 * Math.PI;
        double normalized = angle % fullRotation;

        if (normalized > Math.PI) {
            normalized -= fullRotation;
        }

        if (normalized < -Math.PI) {
            normalized += fullRotation;
        }

        return normalized;
    }


    private double clamp(double position) {

        return Math.max(
                minPosServos,
                Math.min(maxPosServos, position)
        );
    }
    private void moveToPosition(double targetPosition) {

        targetPosition = clamp(targetPosition);
        finalTargetPosition = targetPosition;


        boolean crossHighToLow =
                previousTargetPosition > wrapHigh
                        && targetPosition < wrapLow;

        boolean crossLowToHigh =
                previousTargetPosition < wrapLow
                        && targetPosition > wrapHigh;

        if (!routingThroughMiddle
                && (crossHighToLow || crossLowToHigh)) {

            routingThroughMiddle = true;
        }


        previousTargetPosition = targetPosition;


        double activeTarget = routingThroughMiddle
                ? clamp(safeMiddle)
                : finalTargetPosition;

        double nextPosition = moveToward(
                currentServoPosition,
                activeTarget,
                maxStepPerLoop
        );

        commandPosition(nextPosition);


        if (routingThroughMiddle
                && Math.abs(
                currentServoPosition - clamp(safeMiddle)
        ) < 0.0001) {

            routingThroughMiddle = false;
        }
    }

    private double moveToward(
            double current,
            double target,
            double maxStep
    ) {

        double error = target - current;

        if (Math.abs(error) <= maxStep) {
            return target;
        }


        double dynamicStep = Math.min(
                Math.abs(error) * 0.3,
                maxStep
        );

        return current
                + Math.signum(error) * dynamicStep;
    }

    public void commandPosition(double position) {

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

    public double getTargetHeadingDeg(
            Pose robotPose,
            Pose goalPose
    ) {

        return Math.toDegrees(
                calculateTargetHeading(robotPose, goalPose)
        );
    }
}


