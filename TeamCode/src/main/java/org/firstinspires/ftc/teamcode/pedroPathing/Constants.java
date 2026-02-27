package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.config.hardware.PinpointVisionLocalizer;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.88)
            .forwardZeroPowerAcceleration(-50.67629478691113)
            .lateralZeroPowerAcceleration(-82.11302475953939)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.17, 0, 0.014, 0.035))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.025, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00006, 0.6, 0.04))
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1.15, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(79.11519977239175)
            .yVelocity(62.39379306102362)
            .useBrakeModeInTeleOp(true);
    /*
    123.8 43.24 2.478
    121.81 48.767 -2.4555
     */

    public static PinpointConstants localizerConstants = new PinpointConstants() // NOTE: These values are accurate as of 11/15/2025 except for directions
            .forwardPodY(2.4606299213)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new PinpointVisionLocalizer(hardwareMap, localizerConstants))
                .build();
    }
}
