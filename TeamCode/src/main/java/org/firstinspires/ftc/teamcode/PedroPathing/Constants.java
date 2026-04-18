package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static final double PRIMARY_HEADING_P = 1.08;
    public static final double PRIMARY_HEADING_I = 0;
    public static final double PRIMARY_HEADING_D = 0.03;
    public static final double PRIMARY_HEADING_F = 0.01;

    public static final double SECONDARY_HEADING_P = 0.9;
    public static final double SECONDARY_HEADING_I = 0;
    public static final double SECONDARY_HEADING_D = 0.01;
    public static final double SECONDARY_HEADING_F = 0.01;

    public static final double TELEOP_SECONDARY_HEADING_ERROR_RAD = Math.toRadians(12);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.1)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    PRIMARY_HEADING_P,
                    PRIMARY_HEADING_I,
                    PRIMARY_HEADING_D,
                    PRIMARY_HEADING_F
            ))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    SECONDARY_HEADING_P,
                    SECONDARY_HEADING_I,
                    SECONDARY_HEADING_D,
                    SECONDARY_HEADING_F
            ))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.05838, 0.0015975 ))
            .centripetalScaling(0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("fr")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fl")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.71)
            .yVelocity(65.52);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.825)
            .strafePodX(2.04)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
