//package org.firstinspires.ftc.teamcode.SUBSYSTEMS;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.SubsystemBase; // if I went with SolversLib instead, swap this to its package
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
///**
// * TurretSubsystem
// *
// * Job: aim my turret at the goal using the odometry pose. My swingarm pods go through the goBILDA
// * Pinpoint, and Pedro is already tuned, so I just hand this class the robot pose (x, y, heading)
// * from follower.getPose() every loop. It works out the bearing to the goal, subtracts my heading to
// * get a turret-relative angle, then drives the servos to that angle.
// *
// * Two servos now: "turret servo left" and "turret servo right" both turn the one turret. One is
// * probably mounted mirrored. I do NOT hard-code an "invert" flag, I calibrate each servo's position
// * at the two ends of travel instead. If the right one runs backwards I just swap its two numbers.
// *
// * Open loop on the servos (I command a position, I trust it gets there) because a plain Servo gives
// * me no feedback. If I ever wire up the Axon's analog feedback I can add real closed-loop later.
// *
// * Why update(x,y,heading) instead of grabbing the Follower: keeps this class from caring which Pedro
// * version I'm on. My TeleOp already calls follower.getPose(), so I just pass the numbers in. Less to
// * break while I'm still learning. (It still extends SubsystemBase so it's command-based friendly. If
// * FTCLib ever fights my versions, I can delete "extends SubsystemBase" + that import and it's a plain
// * class, nothing else changes.)
// */
//@Config
//public class TurretSubsystem extends SubsystemBase {
//
//    // ============================================================
//    // FILL IN / TUNE  (live-editable in FTC Dashboard thanks to @Config)
//    // ============================================================
//
//    // ----- MUST-FILL: the two turret servos -----
//    // EXACT names from my Robot Configuration. HEADS UP: both names have SPACES in them. Spaces are
//    // legal but unusual and dead easy to mistype, so I'm flagging it. Double check these match the
//    // Driver Hub config character-for-character (the HUD also warns me if it sees a space).
//    public static String TURRET_SERVO_LEFT_NAME  = "turret servo left";
//    public static String TURRET_SERVO_RIGHT_NAME = "turret servo right";
//
//    // ----- MUST-FILL/TUNE: reachable turret window, measured from robot FORWARD, CCW positive -----
//    // +90 = turret pointing at the robot's LEFT. This window IS my physical travel.
//    // I have a hard blocker at 300 degrees, so MAX - MIN must stay <= 300. Default is a symmetric 300
//    // (so -150 .. +150, i.e. 150 each side of straight ahead). If my blocker is NOT centered on
//    // forward, shift BOTH numbers (e.g. 0 and 300, or -100 and 200). The code clamps to this window,
//    // so it physically cannot try to push past the 300 blocker.
//    public static double TURRET_ANGLE_MIN_DEG = -150.0;     // angle at one mechanical stop
//    public static double TURRET_ANGLE_MAX_DEG =  150.0;     // angle at the other stop (the blocker end)
//    public static double TURRET_ROTATION_LIMIT_DEG = 300.0; // my blocker. HUD warns if my window is wider.
//
//    // ----- MUST-FILL/TUNE: each servo's position at the two window ends -----
//    // At TURRET_ANGLE_MIN_DEG the left servo sits at LEFT_POS_AT_MIN, the right at RIGHT_POS_AT_MIN.
//    // At TURRET_ANGLE_MAX_DEG they sit at the _MAX values. How I find them: nudge a servo to a
//    // position, look at where the turret actually points, repeat until I've pinned both ends.
//    // MIRRORING: if the right servo turns opposite the left, set RIGHT_POS_AT_MIN > RIGHT_POS_AT_MAX
//    // (i.e. swap the two numbers). That's the whole trick, no invert flag needed.
//    public static double LEFT_POS_AT_MIN  = 0.0;   // MUST-FILL
//    public static double LEFT_POS_AT_MAX  = 1.0;   // MUST-FILL
//    public static double RIGHT_POS_AT_MIN = 0.0;   // MUST-FILL  (swap with MAX if it's mirrored)
//    public static double RIGHT_POS_AT_MAX = 1.0;   // MUST-FILL
//
//    // ----- TUNE: how fast the turret spins, in degrees of TURRET per second -----
//    // Positional servos have no "power" dial, so I control speed by limiting how far the commanded
//    // angle is allowed to move each loop. This is that cap. Not full blast, not a crawl: medium.
//    // Bigger = faster. ~120 feels slow-ish, ~250 feels quick. Set it huge (9999) to basically turn the
//    // limit off and snap. Both servos share the same commanded angle, so they always move in sync.
//    public static double SLEW_DEG_PER_SEC = 180.0;
//
//    // ----- MUST-FILL/TUNE: goal field coordinate -----
//    // CRITICAL: same coordinate frame my Pedro/Pinpoint localizer reports (the frame I tuned in:
//    // same origin, same axis directions, same units as follower.getPose(), inches by default). If the
//    // official DECODE numbers use a different origin than my Pedro start pose, I have to translate them
//    // into my frame first or the aim is off by a fixed amount.
//    public static double GOAL_X = 0.0;  // MUST-FILL
//    public static double GOAL_Y = 0.0;  // MUST-FILL
//
//    // ----- TUNE: center/safe angle + "on target" slack -----
//    public static double TURRET_SAFE_CENTER_ANGLE_DEG = 0.0; // where panic/center sends it (forward)
//    public static double ON_TARGET_TOLERANCE_DEG = 2.0;      // open-loop "locked on" slack, degrees
//
//    // ============================================================
//    // internal state (Dashboard ignores non-static fields)
//    // ============================================================
//    private Servo leftServo;
//    private Servo rightServo;
//    private boolean leftPresent;
//    private boolean rightPresent;
//
//    private boolean aiming = true;       // odometry is tuned, so auto-aim is on by default
//    private double cmdAngleDeg;           // the angle I'm currently commanding (this is what the slew moves)
//
//    private final ElapsedTime loopTimer = new ElapsedTime();
//
//    // last-computed values, exposed for the HUD
//    private double lastTargetAngleDeg;
//    private boolean targetWithinRange;    // false => goal is outside my 300 window, I'm parked at a stop
//    private double lastLeftPos;
//    private double lastRightPos;
//    private boolean hasMoved;
//
//    public TurretSubsystem(HardwareMap hardwareMap) {
//        // Guard each grab so a wrong/missing name shows on telemetry instead of crashing init.
//        try {
//            leftServo = hardwareMap.get(Servo.class, TURRET_SERVO_LEFT_NAME);
//            leftPresent = true;
//        } catch (Exception e) {
//            leftServo = null;
//            leftPresent = false;
//        }
//        try {
//            rightServo = hardwareMap.get(Servo.class, TURRET_SERVO_RIGHT_NAME);
//            rightPresent = true;
//        } catch (Exception e) {
//            rightServo = null;
//            rightPresent = false;
//        }
//
//        // Start the commanded angle at center so the turret doesn't lurch on the first loop.
//        cmdAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
//        lastTargetAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
//
//        // NOTE: I do NOT setPosition here on purpose. The servos only get told to move once my TeleOp
//        // starts calling update() in the run loop, which keeps me from moving anything during init.
//        loopTimer.reset();
//    }
//
//    /**
//     * Call this EVERY loop from TeleOp, right after reading the Pedro pose:
//     *
//     *   Pose p = follower.getPose();
//     *   turret.update(p.getX(), p.getY(), p.getHeading());   // heading in radians
//     *
//     * @param robotX          field X of the robot (Pedro frame, inches)
//     * @param robotY          field Y of the robot (Pedro frame, inches)
//     * @param robotHeadingRad robot heading in radians, CCW positive (Pedro convention)
//     */
//    public void update(double robotX, double robotY, double robotHeadingRad) {
//        // dt for the slew. Cap it so the very first loop (or a long stall) can't cause a big jump.
//        double dt = Math.min(loopTimer.seconds(), 0.1);
//        loopTimer.reset();
//
//        // 1) the angle I WANT: aim at the goal, or hold center if I'm not aiming.
//        double desiredAngleDeg;
//        if (aiming) {
//            double dx = GOAL_X - robotX;
//            double dy = GOAL_Y - robotY;
//            double bearingRad = Math.atan2(dy, dx);                     // field bearing to goal, CCW from +X
//            desiredAngleDeg = Math.toDegrees(bearingRad - robotHeadingRad); // turn it into robot-relative
//        } else {
//            desiredAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
//        }
//
//        // 2) pick the representation of that angle closest to my window, then clamp it INTO the window.
//        //    Clamping here is exactly the 300 blocker: I never command past either mechanical stop.
//        double lo = Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
//        double hi = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
//        double fitted = fitToWindow(desiredAngleDeg, lo, hi);
//        targetWithinRange = (fitted >= lo - 1e-9) && (fitted <= hi + 1e-9);
//        double clampedTarget = clamp(fitted, lo, hi);
//        lastTargetAngleDeg = clampedTarget;
//
//        // 3) slew the commanded angle toward the target so the turret moves at my chosen speed.
//        double maxStep = SLEW_DEG_PER_SEC * dt;
//        double err = clampedTarget - cmdAngleDeg;
//        if (Math.abs(err) <= maxStep) {
//            cmdAngleDeg = clampedTarget;          // close enough, just land on it
//        } else {
//            cmdAngleDeg += Math.signum(err) * maxStep; // step toward it
//        }
//
//        // 4) angle -> normalized 0..1 across my travel -> each servo's own position.
//        double span = hi - lo;
//        double u = (span < 1e-9) ? 0.5 : (cmdAngleDeg - lo) / span;
//        u = clamp(u, 0.0, 1.0); // final safety so I never send a servo past an end
//
//        double leftPos  = clamp(lerp(LEFT_POS_AT_MIN,  LEFT_POS_AT_MAX,  u), 0.0, 1.0);
//        double rightPos = clamp(lerp(RIGHT_POS_AT_MIN, RIGHT_POS_AT_MAX, u), 0.0, 1.0);
//        lastLeftPos = leftPos;
//        lastRightPos = rightPos;
//
//        // 5) write to whichever servos are actually present.
//        if (leftPresent)  leftServo.setPosition(leftPos);
//        if (rightPresent) rightServo.setPosition(rightPos);
//        hasMoved = true;
//    }
//
//    // ---- control / safety ----
//
//    /** Panic + safe: stop aiming and glide the turret back to center at the normal spin speed. */
//    public void goToSafeCenter() {
//        aiming = false; // target becomes center, the slew carries it there smoothly
//    }
//
//    /** Same idea but instant (no glide) for a true panic stop. Snaps the commanded angle to center now. */
//    public void snapToCenterNow() {
//        aiming = false;
//        cmdAngleDeg = TURRET_SAFE_CENTER_ANGLE_DEG;
//        double lo = Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
//        double hi = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
//        double span = hi - lo;
//        double u = (span < 1e-9) ? 0.5 : clamp((cmdAngleDeg - lo) / span, 0.0, 1.0);
//        if (leftPresent)  leftServo.setPosition(clamp(lerp(LEFT_POS_AT_MIN,  LEFT_POS_AT_MAX,  u), 0.0, 1.0));
//        if (rightPresent) rightServo.setPosition(clamp(lerp(RIGHT_POS_AT_MIN, RIGHT_POS_AT_MAX, u), 0.0, 1.0));
//    }
//
//    public void enableAutoAim()  { aiming = true; }
//    public void disableAutoAim() { aiming = false; }
//
//    // ---- getters for the HUD ----
//    public boolean isAiming()             { return aiming; }
//    public boolean isLeftPresent()        { return leftPresent; }
//    public boolean isRightPresent()       { return rightPresent; }
//    public boolean leftNameHasSpace()     { return TURRET_SERVO_LEFT_NAME.contains(" "); }
//    public boolean rightNameHasSpace()    { return TURRET_SERVO_RIGHT_NAME.contains(" "); }
//    public double  getTargetAngleDeg()    { return lastTargetAngleDeg; }
//    public double  getCommandedAngleDeg() { return cmdAngleDeg; }
//    public double  getLeftPosition()      { return lastLeftPos; }
//    public double  getRightPosition()     { return lastRightPos; }
//    public boolean isWithinRange()        { return targetWithinRange; } // false => goal is in my dead zone
//    /** True if my configured window is wider than my real 300 blocker, which would be a calibration mistake. */
//    public boolean windowExceedsBlocker() {
//        double width = Math.max(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG)
//                - Math.min(TURRET_ANGLE_MIN_DEG, TURRET_ANGLE_MAX_DEG);
//        return width > TURRET_ROTATION_LIMIT_DEG + 1e-6;
//    }
//    /** Open-loop "locked on": aiming, the goal is reachable, and the slew has basically caught up. */
//    public boolean isOnTarget() {
//        return aiming && targetWithinRange
//                && Math.abs(lastTargetAngleDeg - cmdAngleDeg) <= ON_TARGET_TOLERANCE_DEG;
//    }
//
//    // ---- tiny math helpers ----
//
//    /** Pick the version of angleDeg (+/- 360) that sits closest to the window, so wraparound behaves. */
//    private static double fitToWindow(double angleDeg, double lo, double hi) {
//        double center = 0.5 * (lo + hi);
//        double a = angleDeg;
//        while (a - center >  180.0) a -= 360.0;
//        while (a - center < -180.0) a += 360.0;
//        return a; // may still be outside [lo, hi]; the caller clamps and flags it
//    }
//
//    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
//
//    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
//}