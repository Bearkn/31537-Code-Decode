package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class RED extends OpMode {
//    MecanumDrive drive = new MecanumDrive();
    Turret turret = new Turret();

    double tolerance = .05;
    double x,y,turn;

    @Override
    public void init() {
//        drive.init(hardwareMap);
        turret.init(hardwareMap);

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        if (y >= -tolerance && y <= tolerance) {
            y = 0;
        }
        if (x >= -tolerance && x <= tolerance) {
            x = 0;
        }
        if (turn >= -tolerance && turn <= .1) {
            turn = 0;
        }
    }

    @Override
    public void loop(){

        if(gamepad1.bWasPressed()){
            turret.turretAngle += 10;
        }
        if(gamepad1.aWasPressed()){
            turret.turretAngle -= 10;
        }
        turret.update();
//        if(vision.llResult != null && vision.llResult.isValid()) {
////            Pose3D botPoseMt2 = llResult.getBotpose_MT2();
//            telemetry.addData("tx", vision.llResult.getTx());
//            telemetry.addData("ty", vision.llResult.getTy());
//            telemetry.addData("ta", vision.llResult.getTa());
//            shoot.speedCalc(vision.llResult.getTy());
//
//        }


        telemetry.addData("turret posF",turret.turretServoFront.getPosition());
        telemetry.addData("turret posB",turret.turretServoBack.getPosition());
        telemetry.addData("turret posF",turret.turretAngle);
        telemetry.addData("turret posB",turret.turretAngle);

        telemetry.update();
//        drive.driveFieldRelative(y,x,turn);
    }
}
