package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class TurretSubsystemAaravDewan extends SubsystemBase {

    public static String TURRET_SERVO_LEFT_NAME  = "turretservoleft";
    public static String TURRET_SERVO_RIGHT_NAME = "turretservoright";

    public static double TURRET_ANGLE_MIN_DEG = -150.0;
    public static double TURRET_ANGLE_MAX_DEG =  150.0;
    public static double TURRET_ROTATION_LIMIT_DEG = 300.0;

    public static double LEFT_POS_AT_MIN  = 0.0;
    public static double LEFT_POS_AT_MAX  = 1.0;
    public static double RIGHT_POS_AT_MIN = 0.0;
    public static double RIGHT_POS_AT_MAX = 1.0;

    public static double SLEW_DEG_PER_SEC = 180.0;

    public static double RED_GOAL_X  = 144.0;
    public static double RED_GOAL_Y  = 144.0;
    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 144.0;

    public static double TURRET_SAFE_CENTER_ANGLE_DEG = 0.0;
    public static double ON_TARGET_TOLERANCE_DEG = 2.0;

    private Servo leftServo;
    private Servo rightServo;
    private boolean leftPresent;
    private boolean rightPresent;

    private boolean aiming = true;
    private boolean redAlliance = true;
    private double cmdAngleDeg;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double lastTargetAngleDeg;
    private boolean targetWithinRange;
    private double lastLeftPos;
    private double lastRightPos;

    public TurretSubsystemAaravDewan (HardwareMap hardwareMap) {
        try {
            leftServo = hardwareMap.get(Servo.class, TURRET_SERVO_LEFT_NAME);
            leftPresent = true;
        } catch (Exception e) {
            leftServo = null;
            leftPresent = false;
        }
        try {
            rightServo = hardwareMap.get(Servo.class, TURRET_SERVO_RIGHT_NAME);
            rightPresent = true;
        } catch (Exception e) {
            rightServo = null;
            rightPresent = false;
        }

        cmdAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
        lastTargetAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
        loopTimer.reset();
    }

    public void update(double robotX, double robotY, double robotHeadingRad) {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        double goalX = redAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = redAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

        double desiredAngleDeg;
        if (aiming) {
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double bearingRad = Math.atan2(dy, dx);
            desiredAngleDeg = angleWrap(Math.toDegrees(bearingRad - robotHeadingRad));        } else {
            desiredAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
        }

        double lo = Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
        double hi = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
        double fitted = fitToWindow(desiredAngleDeg, lo, hi);
        targetWithinRange = (fitted >= lo - 1e-9) && (fitted <= hi + 1e-9);
        double clampedTarget = clamp(fitted, lo, hi);
        lastTargetAngleDeg = clampedTarget;

        double maxStep = SLEW_DEG_PER_SEC * dt;
        double err = angleWrap(clampedTarget - cmdAngleDeg);
        if (Math.abs(err) <= maxStep) {
            cmdAngleDeg = clampedTarget;
        } else {
            cmdAngleDeg += Math.signum(err) * maxStep;
        }

        double span = hi - lo;
        double u = (span < 1e-9) ? 0.5 : (cmdAngleDeg - lo) / span;
        u = clamp(u, 0.0, 1.0);

        double leftPos  = clamp(lerp(LEFT_POS_AT_MIN,  LEFT_POS_AT_MAX,  u), 0.0, 1.0);
        double rightPos = clamp(lerp(RIGHT_POS_AT_MIN, RIGHT_POS_AT_MAX, u), 0.0, 1.0);
        lastLeftPos = leftPos;
        lastRightPos = rightPos;

        if (leftPresent)  leftServo.setPosition(leftPos);
        if (rightPresent) rightServo.setPosition(rightPos);
    }

    public void goToSafeCenter() {
        aiming = false;
    }

    public void snapToCenterNow() {
        aiming = false;
        cmdAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
        double lo = Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
        double hi = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
        double span = hi - lo;
        double u = (span < 1e-9) ? 0.5 : clamp((cmdAngleDeg - lo) / span, 0.0, 1.0);
        if (leftPresent)  leftServo.setPosition(clamp(lerp(LEFT_POS_AT_MIN,  LEFT_POS_AT_MAX,  u), 0.0, 1.0));
        if (rightPresent) rightServo.setPosition(clamp(lerp(RIGHT_POS_AT_MIN, RIGHT_POS_AT_MAX, u), 0.0, 1.0));
    }

    public void enableAutoAim()  { aiming = true; }
    public void disableAutoAim() { aiming = false; }

    public void setAlliance(boolean isRed) { redAlliance = isRed; }
    public void aimAtRedGoal()  { redAlliance = true; }
    public void aimAtBlueGoal() { redAlliance = false; }

    public boolean isAiming()             { return aiming; }
    public boolean isRedAlliance()        { return redAlliance; }
    public boolean isLeftPresent()        { return leftPresent; }
    public boolean isRightPresent()       { return rightPresent; }
    public double  getTargetAngleDeg()    { return lastTargetAngleDeg; }
    public double  getCommandedAngleDeg() { return cmdAngleDeg; }
    public double  getLeftPosition()      { return lastLeftPos; }
    public double  getRightPosition()     { return lastRightPos; }
    public boolean isWithinRange()        { return targetWithinRange; }

    public boolean windowExceedsBlocker() {
        double width = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG)
                - Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
        return width > TURRET_ROTATION_LIMIT_DEG + 1e-6;
    }

    public boolean isOnTarget() {
        return aiming && targetWithinRange
                && Math.abs(lastTargetAngleDeg - cmdAngleDeg) <= ON_TARGET_TOLERANCE_DEG;
    }

    private static double fitToWindow(double angleDeg, double lo, double hi) {
        double center = 0.5 * (lo + hi);
        double a = angleDeg;
        while (a - center >  180.0) a -= 360.0;
        while (a - center < -180.0) a += 360.0;
        return a;
    }
    private static double angleWrap(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}