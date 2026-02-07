package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.20163d)
            .forwardZeroPowerAcceleration(-32.917d)
            .lateralZeroPowerAcceleration(-61.28d)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.01,
                    0,
                    0.02,
                    0.005
            ))
//            .translationalPIDFSwitch(4)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.4,
//                    0,
//                    0.005,
//                    0.0006
//            ))
//            .headingPIDFCoefficients(new PIDFCoefficients(
//                    1,
//                    0,
//                    0.03,
//                    0.03
//            ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
//                    2.5,
//                    0,
//                    0.1,
//                    0.0005
//            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.02,
//                    0,
//                    0.000005,
//                    0.6,
//                    0.01
//            ))
//            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.90, 100, .8, .5);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frm")
            .rightRearMotorName("brm")
            .leftRearMotorName("blm")
            .leftFrontMotorName("flm")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(74.652)
            .yVelocity(63.333);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.93701) // .25
            .strafePodX(-5.560393701) // -.85
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
