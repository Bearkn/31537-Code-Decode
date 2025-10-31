package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public GoBildaPinpointDriver imu;

    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class, "flm");
        frontRightMotor = hwMap.get(DcMotor.class, "frm");
        backLeftMotor = hwMap.get(DcMotor.class, "blm");
        backRightMotor = hwMap.get(DcMotor.class, "brm");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu.setHeading(0,AngleUnit.DEGREES);
        imu.recalibrateIMU();



//        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
//
//
//        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void drive(double power, double theta, double turn) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        double leftFront = power * cos/max + turn;
        double rightFront = power * sin/max - turn;
        double leftRear = power * sin/max + turn;
        double rightRear = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1){
            leftFront   /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftRear    /= power + Math.abs(turn);
            rightRear  /= power + Math.abs(turn);
        }

        frontLeftMotor.setPower(leftFront);
        frontRightMotor.setPower(rightFront);
        backLeftMotor.setPower(leftRear);
        backRightMotor.setPower(rightRear);
    }

    public void driveFieldRelative(double y, double x, double turn){
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);
        imu.update();
        theta = AngleUnit.normalizeRadians(theta - imu.getHeading(AngleUnit.RADIANS));

//        double newForward = r * Math.sin(theta);
//        double newStrafe = r * Math.cos(theta);

        this.drive(r,theta,turn);
    }
    public void driveRobotRelative(double y, double x, double turn){
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);

        this.drive(r,theta,turn);
    }
}
