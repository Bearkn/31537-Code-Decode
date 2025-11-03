package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class RED extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(24, -64, 0); // Start Pose of our robot.
    MecanumDrive drive = new MecanumDrive();
    Shooter shoot = new Shooter();
    Intake intake = new Intake();

    boolean spinshooter = false;

    int turretActivated = 0;
//    GoalAprilTagTracker tracker = new GoalAprilTagTracker();

//    boolean autoaim = false;

    double tolerance = .05;

    double forward, strafe, rotate;

    @Override
    public void init(){
        drive.init(hardwareMap);
//        tracker.init(hardwareMap);
        intake.init(hardwareMap);
        shoot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop(){
        if(turretActivated == 0) {
            shoot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoot.spinTurret(shoot.turretAngle(true));
        } else if (turretActivated == 1) {
            shoot.speed = 1550;
            shoot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoot.turretMotor.setPower(0);
            shoot.spinTurret(0);
        } else if (turretActivated == 2) {
            shoot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoot.turretMotor.setPower(0);
        }
        follower.update();
//
        if(gamepad1.left_bumper){

//            shoot.flyWheelActivated = !shoot.flyWheelActivated;
            if(Math.abs(shoot.flyWheelMotor1.getVelocity() - shoot.speed) < 30) {
                intake.turretSpinIntake = true;
                intake.hardStopActivated = true;
            }
        } else {
                intake.turretSpinIntake = false;
            intake.hardStopActivated = false;
            }

//        if(shoot.flyWheelActivated){
////            shoot.speed = 1500;
//            if(Math.abs(shoot.flyWheelMotor1.getVelocity() - shoot.speed) < 30) {
//                intake.turretSpinIntake = true;
//                intake.hardStopActivated = true;
//            }
//        } else {
//            intake.turretSpinIntake = false;
//            intake.hardStopActivated = false;
//        }

        if(gamepad1.dpadDownWasPressed()){
            spinshooter = !spinshooter;
        }

        if(spinshooter){
            shoot.spinShooter = true;
        } else {
            shoot.spinShooter = false;
        }
        if(gamepad1.rightBumperWasPressed()){
            intake.speed = .9;
            intake.spinIntake = !intake.spinIntake;
        }

        intake.reverseIntake = gamepad1.a;

        if(gamepad1.xWasPressed()){
            shoot.spinShooter = !shoot.spinShooter;
//            shoot.speed += .01;
        }

        if(gamepad1.dpadDownWasPressed()){
            shoot.teamColor = !shoot.teamColor;
        }
        if(gamepad1.dpadUpWasPressed()){
            turretActivated += 1;
            if(turretActivated > 3){
                turretActivated = 0;
            }
        }

        if(gamepad1.yWasPressed()){
//            shoot.speed -= .05;
            intake.setServoPos(.6);

        }

        if(gamepad1.dpadLeftWasPressed()){
            shoot.speedChange += 500;
        }

        if(gamepad1.dpadRightWasPressed()){
            shoot.speedChange -= 500;

        }
        intake.hardStopPos(intake.hardStopActivated);
        intake.spin();
        shoot.spin();
        double y = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double turn  = gamepad1.right_stick_x;
        if (y >= -tolerance && y <= tolerance) {
            y = 0;
        }
        if (x >= -tolerance && x <= tolerance) {
            x = 0;
        }
        if (turn >= -tolerance && turn <= .1) {
            turn = 0;
        }
//        tracker.update();
        telemetry.addData("Heading", drive.imu.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:",follower.getPose().getY());
        telemetry.addData("servo Angle",intake.hardStop.getPosition());
        telemetry.addData("intakeOn",intake.spinIntake);
        telemetry.addData("shooterOn",shoot.spinShooter);
        telemetry.addData("shooterSpeed",shoot.speed);
        telemetry.addData("turret rotation",shoot.angleNormalize(shoot.turretMotor.getCurrentPosition()));
        telemetry.addData("shooter rpm",shoot.flyWheelMotor1.getVelocity());
        telemetry.addData("angle from goal",shoot.turretAngle(true));
        telemetry.addData("turret actived",turretActivated);
        telemetry.addData("shoot",shoot.shooting);
        telemetry.addData("spinshooter",shoot.spinShooter);
        telemetry.addData("hardstop", intake.hardStopActivated);
        telemetry.addData("turretintakespin", intake.turretSpinIntake);
        telemetry.addData("Are we Red Team", shoot.teamColor);
        telemetry.addData("speed of shooter", shoot.speedChange);





        telemetry.update();

//        telemetry.addData("error", shoot.spinTurret(60));







//        if(!tracker.detections.isEmpty() & autoaim) {
//            telemetry.addData("x", tracker.tag.ftcPose.x);
//            telemetry.addData("y", tracker.tag.ftcPose.y);
//            telemetry.addData("z", tracker.tag.ftcPose.z);
//            telemetry.addData("roll", tracker.tag.ftcPose.roll);
//            telemetry.addData("pitch", tracker.tag.ftcPose.pitch);
//            telemetry.addData("yaw", tracker.tag.ftcPose.yaw);
//
//            if(tracker.tag.ftcPose.x >= 1 || tracker.tag.ftcPose.x <= -1){
//                drive.driveFieldRelative(0,0, MathFunctions.clamp(tracker.tag.ftcPose.x/20,-.4,.4));
//            } else if (Math.abs(tracker.tag.ftcPose.x) < 1 ){
//                if(tracker.tag.ftcPose.y >20){
//                    drive.driveRobotRelative(.2,0,0);
//                }
//                else{
//                    drive.driveRobotRelative(0,0,0);
//                }
//            }
//
//
//        } else {
//            drive.driveFieldRelative(y,x,turn);
//        }


//        drive.driveRobotRelative(y,x,turn);
         drive.driveFieldRelative(y,x,turn);
    }
}
