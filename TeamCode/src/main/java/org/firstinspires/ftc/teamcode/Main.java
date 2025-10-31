package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.GoalAprilTagTracker;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;

@TeleOp
public class Main extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Shooter shoot = new Shooter();
    Intake intake = new Intake();
    GoalAprilTagTracker tracker = new GoalAprilTagTracker();

    boolean autoaim = false;

    double forward, strafe, rotate;

    @Override
    public void init(){
        drive.init(hardwareMap);
        tracker.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop(){

        if(gamepad1.bWasPressed()){
            shoot.spin(.05);
        } else {
            shoot.spin(0);
        }

        if(gamepad1.aWasPressed()){
            intake.spin(.2);
        } else {
            intake.spin(0);
        }
        double y = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double turn  = gamepad1.right_stick_x;
        tracker.update();
        telemetry.addData("Heading", drive.imu.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x_drive", x);
        telemetry.addData("y_drive", y);
        telemetry.addData("t_drive", turn);
        telemetry.addData("servo Angle",intake.hardStop.getPosition());

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


        drive.driveRobotRelative(y,x,turn);
//         drive.driveFieldRelative(y,x,turn);
    }
}
