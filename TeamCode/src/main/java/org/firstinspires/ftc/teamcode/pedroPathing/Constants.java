package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.hardware.PinpointVisionLocalizer;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.363)
            .forwardZeroPowerAcceleration(-54.3990082915146)
            .lateralZeroPowerAcceleration(-81.9856678792281)

            //.translationalPIDFCoefficients(new PIDFCoefficients(0.34, 0, 0.024, 0.032))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.03))

            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.00024, 0.6, 0.06))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.03860803441578884, 0.0022273191574803766))
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(68.18830066590797)
            .yVelocity(59.63105016993726)
            .useBrakeModeInTeleOp(true);
    /*
    123.8 43.24 2.478
    121.81 48.767 -2.4555
     */

    public static PinpointConstants localizerConstants = new PinpointConstants() // NOTE: These values are accurate as of 11/15/2025 except for directions
            .forwardPodY(-2.46063)
            .strafePodX(-3.562992)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MyFusionLocalizer fusionLocalizer;

    public static Follower createFollower(HardwareMap hardwareMap) {
        fusionLocalizer = new MyFusionLocalizer(
                new PinpointLocalizer(hardwareMap, localizerConstants),
                new Pose(0.25, 0.25, Math.toRadians(2)),
                new Pose(0.002, 0.002, Math.toRadians(0.1)),
                new Pose(0.5, 0.5, Math.toRadians(1)),
                50
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(fusionLocalizer)
                .build();
    }
}
