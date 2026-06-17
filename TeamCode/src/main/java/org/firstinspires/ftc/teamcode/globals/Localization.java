package org.firstinspires.ftc.teamcode.globals;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Localization {
    private static Follower follower;
    private static final ElapsedTime timer = new ElapsedTime();
    private static double lastHeading = 0.0;     // rad
    private static double headingVel = 0.0;      // rad/s (filtered)

    public static void init(Follower f) {
        follower = f;

        follower.update();
        lastHeading = follower.getHeading();
        headingVel = 0.0;
        timer.reset();
    }

    public static void update() {
        follower.update();
        headingVel = follower.getAngularVelocity(); // rad/s, no extra lag
        lastHeading = follower.getHeading();
    }

    public static double getHeading() {
        return follower.getHeading();
    }

    public static double getHeadingVelocity() {
        return headingVel;
    }

    public static double getX() {
        return follower.getPose().getX();
    }

    public static Vector getVelocity() { return follower.getVelocity();}

    public static Pose getPose() {return follower.getPose();}
    public static double getDistance() {
        return RobotConstants.redGoalPose.distanceFrom(getPose());
    }

    public static double getZLateral() {
        return follower.getPose().getY();
    }

}