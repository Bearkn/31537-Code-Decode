package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class RED extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Turret turret = new Turret();

    Shooter shooter = new Shooter();

//    Intake intake = new Intake();

    // control booleans



    double tolerance = .03;
    double x,y,turn;

    @Override
    public void init() {
        drive.init(hardwareMap);
        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        Intake.init(hardwareMap);



    }

    @Override
    public void loop(){
        drive.imu.update();

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

        if(gamepad1.rightBumperWasPressed()){
            Intake.intakeOn = !Intake.intakeOn;
        }

        if(!Intake.Outtake) {
            if (Intake.intakeOn) {
                Intake.intakeState = Intake.IntakeState.INTAKE;

            } else {
                Intake.intakeState = Intake.IntakeState.STOP;
            }
        } else {
            Intake.intakeState = Intake.IntakeState.OUTTAKE;
            Intake.indexState = Intake.IndexState.OUTTAKE;
        }

        if(gamepad1.leftBumperWasPressed()){
            Intake.stopOn = !Intake.stopOn;
        }

        if(Intake.stopOn){
            Intake.stopState = Intake.StopState.HOLD;
            Intake.indexState = Intake.IndexState.STOP;

        } else {
            Intake.stopState = Intake.StopState.SHOOT;
            Intake.indexState = Intake.IndexState.INTAKE;

        }

//        if(gamepad1.bWasPressed()){
//            turret.turretAngle += 10;
//        }
//        if(gamepad1.aWasPressed()){
//            turret.turretAngle -= 10;
//        }

        if(gamepad1.aWasPressed()){
            shooter.shooterActivated = !shooter.shooterActivated;
        }


//        if(gamepad1.bWasPressed()){
//            shooter.stepIndex = (shooter.stepIndex + 1) % shooter.stepsizes.length;
//        }
//
//        if(gamepad1.dpadLeftWasPressed()){
//            shooter.Kf += shooter.stepsizes[shooter.stepIndex];
//        }
//
//        if(gamepad1.dpadRightWasPressed()){
//            shooter.Kf -= shooter.stepsizes[shooter.stepIndex];
//        }
//
//        if(gamepad1.dpadUpWasPressed()){
//            shooter.Kp += shooter.stepsizes[shooter.stepIndex];
//        }
//
//        if(gamepad1.dpadDownWasPressed()){
//            shooter.Kp -= shooter.stepsizes[shooter.stepIndex];
//        }
//
        if(gamepad1.dpadDownWasPressed()){
            shooter.hoodAngle +=.01;
        }
        if(gamepad1.dpadUpWasPressed()){
            shooter.hoodAngle -= .01;
        }

        if(gamepad1.dpadLeftWasPressed()){
            shooter.targetFlywheelSpeed += 10;
        }
        if(gamepad1.dpadRightWasPressed()){
            shooter.targetFlywheelSpeed -= 10;
        }


//        turret.turretAngle = drive.imu.getHeading(AngleUnit.DEGREES);

        turret.update();
        shooter.update();
        Intake.update();
//        if(vision.llResult != null && vision.llResult.isValid()) {
////            Pose3D botPoseMt2 = llResult.getBotpose_MT2();
//            telemetry.addData("tx", vision.llResult.getTx());
//            telemetry.addData("ty", vision.llResult.getTy());
//            telemetry.addData("ta", vision.llResult.getTa());
//            shoot.speedCalc(vision.llResult.getTy());
//
//        }

        telemetry.addData("stop",Intake.stopState);
        telemetry.addData("turret posF",turret.turretServoFront.getPosition());
        telemetry.addData("turret posB",turret.turretServoBack.getPosition());
        telemetry.addData("turret posF",turret.turretAngle);
        telemetry.addData("turret posB",turret.turretAngle);

        telemetry.addData("target velo", shooter.targetFlywheelSpeed);
        telemetry.addData("shooter power", shooter.power);
        telemetry.addData("current velo", shooter.currentFlywheelSpeed);
        telemetry.addData("tuning P", "%.5f",shooter.Kp);
        telemetry.addData("tuning F", "%.5f",shooter.Kf);
        telemetry.addData("Step Size", "%.5f",shooter.stepsizes[shooter.stepIndex]);
        telemetry.addData("hoodAngle", shooter.hoodAngle);
        telemetry.addData("currentHOodANgle", shooter.hood.getPosition());

        telemetry.addData("imu angle", turret.turretAngle);


        telemetry.update();
        drive.driveFieldRelative(y,x,turn);
    }
}
