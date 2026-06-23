package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Config
public class RobotConstants {


    public static Pose blueGoalPose =
            new Pose(
                    1.1399688958009242,//0.9844479004665576,
                    134.49766718506996,//131.72706065318818,
                    Math.toRadians(90)
            );

    public static Pose redGoalPose =
            new Pose(
                    135,
                    138,
                    Math.toRadians(90)
            );

    public static Pose redDistancePose =
            new Pose(
                    141,
                    141,
                    Math.toRadians(90)
            );

    public static String chosenAlliance = "RED";

    public static Pose savedPose = null;


    public static double distanceBiasInches = 0.0;

    /*
     * ==========================================================
     * SHOOTER INTERPOLATION TABLE
     * ==========================================================
     *
     * DISTANCE = distance to the goal in inches
     * VELOCITY = shooter motor velocity in ticks per second
     * HOOD     = ServoEx logical servo position
     *
     */

    public static double DISTANCE_0_IN = 24.0;
    public static double VELOCITY_0_TPS = 900.0;
    public static double HOOD_0 = 0.72;

    public static double DISTANCE_1_IN = 36.0;
    public static double VELOCITY_1_TPS = 980.0;
    public static double HOOD_1 = 0.68;

    public static double DISTANCE_2_IN = 48.0;
    public static double VELOCITY_2_TPS = 1060.0;
    public static double HOOD_2 = 0.63;

    public static double DISTANCE_3_IN = 60.0;
    public static double VELOCITY_3_TPS = 1140.0;
    public static double HOOD_3 = 0.58;

    public static double DISTANCE_4_IN = 72.0;
    public static double VELOCITY_4_TPS = 1230.0;
    public static double HOOD_4 = 0.53;

    public static double DISTANCE_5_IN = 84.0;
    public static double VELOCITY_5_TPS = 1320.0;
    public static double HOOD_5 = 0.48;

    public static double DISTANCE_6_IN = 96.0;
    public static double VELOCITY_6_TPS = 1420.0;
    public static double HOOD_6 = 0.43;

    public static double DISTANCE_7_IN = 108.0;
    public static double VELOCITY_7_TPS = 1530.0;
    public static double HOOD_7 = 0.39;

    public static double[] getShooterDistanceTable() {

        return new double[]{
                DISTANCE_0_IN,
                DISTANCE_1_IN,
                DISTANCE_2_IN,
                DISTANCE_3_IN,
                DISTANCE_4_IN,
                DISTANCE_5_IN,
                DISTANCE_6_IN,
                DISTANCE_7_IN
        };
    }


    public static double[] getShooterVelocityTable() {

        return new double[]{
                VELOCITY_0_TPS,
                VELOCITY_1_TPS,
                VELOCITY_2_TPS,
                VELOCITY_3_TPS,
                VELOCITY_4_TPS,
                VELOCITY_5_TPS,
                VELOCITY_6_TPS,
                VELOCITY_7_TPS
        };
    }

    public static double[] getShooterHoodTable() {

        return new double[]{
                HOOD_0,
                HOOD_1,
                HOOD_2,
                HOOD_3,
                HOOD_4,
                HOOD_5,
                HOOD_6,
                HOOD_7
        };
    }
}