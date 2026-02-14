package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Algs.FileManager;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;

@TeleOp
public class blue extends OpMode {
    private Follower follower;
//    private Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose startPose = new Pose(33.000-72, 136.000-72, Math.toRadians(270)); // Start Pose of our robot.

    MecanumDrive drive = new MecanumDrive();
    Turret turret = new Turret();

    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    FileManager fileManager = new FileManager();


    // control booleans



    double tolerance = .03;
    double x,y,turn;

    @Override
    public void init() {
        drive.init(hardwareMap);
        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
        fileManager.init();
//        fileManager.FileWrite(follower.getPose().getX(),24,follower.getHeading());
        fileManager.FileRead();
        telemetry.addData("points",fileManager.routine);
        Pose Autonpose = new Pose(fileManager.routine.get(0),fileManager.routine.get(1),fileManager.routine.get(2));
        follower.setPose(Autonpose);





    }

    @Override
    public void loop(){
        drive.imu.update();

        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
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
            intake.intakeOn = !intake.intakeOn;
        }

        intake.Outtake = (gamepad1.left_trigger > .5);
        intake.stopOn = !(gamepad1.right_trigger > .3);


        if(!intake.Outtake) {
            if (intake.intakeOn) {
                intake.intakeState = Intake.IntakeState.INTAKE;
                intake.indexState = Intake.IndexState.INTAKE;

            } else {
                intake.intakeState = Intake.IntakeState.STOP;
                intake.indexState = Intake.IndexState.STOP;
            }
        } else {
            intake.intakeState = Intake.IntakeState.OUTTAKE;
            intake.indexState = Intake.IndexState.OUTTAKE;
        }

        intake.stopOn = !(gamepad1.right_trigger > .3);


        if(!intake.stopOn){
            if(!intake.intakeOn){
                intake.indexState = Intake.IndexState.INTAKE;
                intake.intakeState = Intake.IntakeState.SHOOT;
            }
            if(shooter.currentFlywheelSpeed >= Math.abs(shooter.targetFlywheelSpeed-75)) {
                intake.stopState = Intake.StopState.SHOOT;
            }
        } else {
            intake.stopState = Intake.StopState.HOLD;
        }

        if(gamepad1.dpad_left){
            follower.setPose(new Pose (62.5,-63.5,Math.toRadians(0)));
        }
        if(gamepad1.dpad_right){
            follower.setPose(new Pose (-49.5,55.5,follower.getHeading()));
        }
        if(gamepad1.dpadDownWasPressed()) {
            turret.offset -= .005;
        }

        if(gamepad1.dpadUpWasPressed()) {
            turret.offset += .005;
        }

        if(gamepad1.leftBumperWasPressed()){
            shooter.shooterActivated = !shooter.shooterActivated;
        }

        if(shooter.shooterActivated){
            gamepad1.rumble(100);
        }

        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.blueGoalX,turret.blueGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),false,true);
        shooter.update(Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.blueGoalX,turret.blueGoalY),shooter.currentFlywheelSpeed);
        intake.update();
        follower.update();
        turret.FFturret(follower.getHeading());



        telemetry.addData("robotX velo", follower.getVelocity().getXComponent());
        telemetry.addData("robotY velo", follower.getVelocity().getYComponent());
        telemetry.addData("robot turn speed", turret.AngularVelocity(follower.getHeading()));
        telemetry.addData("FF", turret.turretFeedForwardServo);
        telemetry.addData("FF tuning", turret.TURRET_FF_GAIN);

        telemetry.addData("missangle",Math.abs(turret.analogangle - (turret.turretAngle*360) ));



        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:",follower.getPose().getY());

        telemetry.addData("turret x", turret.turretpositionX(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()));
        telemetry.addData("turret y", turret.turretpositionY(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()));

        telemetry.addData("intake",intake.intakeState);
        telemetry.addData("intake speed",intake.targetIntakeSpeed);
        telemetry.addData("turret servo position",turret.turretServoBack.getPosition());
        telemetry.addData("hardstop state",intake.stopState);

//        telemetry.addData("turret servo analog",turret.turretEncoder.getAngleUnit());
        telemetry.addData("turret posF",turret.turretAngle);
        telemetry.addData("turret posB",turret.turretAngle);

        telemetry.addData("target velo", shooter.targetFlywheelSpeed);
        telemetry.addData("shooter power", shooter.power);
        telemetry.addData("current velo", shooter.currentFlywheelSpeed);
        telemetry.addData("target velo", intake.targetIntakeSpeed);
        telemetry.addData("shooter power", intake.power);
        telemetry.addData("current velo", intake.currentIntakeSpeed);
        telemetry.addData("tuning P", "%.5f",shooter.Kp);
        telemetry.addData("tuning F", "%.5f",shooter.Kf);
        telemetry.addData("Step Size", "%.5f",shooter.stepsizes[shooter.stepIndex]);
        telemetry.addData("hoodAngle", shooter.hoodAngle);
        telemetry.addData("currentHOodANgle", shooter.hood.getPosition());

//        telemetry.addData("for servo angle", MathFunctions.normalizeAngle(turret.angleToUnit(turret.turretAngle-180)));
        telemetry.addData("turret angle", turret.turretAngle);
        telemetry.addData("field angle", turret.fieldAngle);
        telemetry.addData("distance", Shooter.distance2D(follower.getPose().getX(), follower.getPose().getY(), turret.blueGoalX,turret.blueGoalY));
        telemetry.addData("blue Goal X", turret.blueGoalX);
        telemetry.addData("blue goal Y", turret.blueGoalY);

        telemetry.addData("analog", turret.analogangle);
        telemetry.addData("pos", turret.turretpos);

        telemetry.addData("index speed", intake.intakeR.getVelocity());

        telemetry.addData("outtake", intake.Outtake);









        telemetry.update();
        drive.driveFieldRelative(y,x,turn);
    }
}
