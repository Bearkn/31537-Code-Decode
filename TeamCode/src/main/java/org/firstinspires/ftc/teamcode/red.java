package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class red extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    MecanumDrive drive = new MecanumDrive();
    Turret turret = new Turret();

    Shooter shooter = new Shooter();

    Intake intake = new Intake();


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
        follower.setStartingPose(startPose);



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
            intake.intakeOn = !intake.intakeOn;
        }

        gamepad1.a = intake.Outtake;


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
//            if(shooter.currentFlywheelSpeed >= (shooter.targetFlywheelSpeed-200)) {
                intake.stopState = Intake.StopState.SHOOT;
//            }
        } else {
                intake.stopState = Intake.StopState.HOLD;
        }

//        if(gamepad1.dpad_left){
//            follower.setPose(new Pose (-63.306,-59.74,Math.toRadians(180)));
//        }

//        63.306 0
//
//        if(gamepad1.bWasPressed()){
//            turret.turretpos += .01;
//        }
//        if(gamepad1.xWasPressed()){
//            turret.turretpos -= .01;
//        }

        if(gamepad1.leftBumperWasPressed()){
            shooter.shooterActivated = !shooter.shooterActivated;
        }
//
//
//        if(gamepad1.bWasPressed()){
//            shooter.stepIndex = (shooter.stepIndex + 1) % shooter.stepsizes.length;
//        }

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

//        if(gamepad1.dpadLeftWasPressed()){
//            turret.blueGoalX += .5;
//        }
//
//        if(gamepad1.dpadRightWasPressed()){
//            turret.blueGoalX -= .5;
//        }
//
//        if(gamepad1.dpadUpWasPressed()){
//            turret.blueGoalY += .5;
//        }
//
//        if(gamepad1.dpadDownWasPressed()){
//            turret.blueGoalY -= .5;
//        }

        if(gamepad1.dpadDownWasPressed()){
            shooter.hoodAngle +=.01;
        }
        if(gamepad1.dpadUpWasPressed()){
            shooter.hoodAngle -= .01;
        }

        if(gamepad1.dpadLeftWasPressed()){
            turret.TURRET_FF_GAIN += .01;
        }
        if(gamepad1.dpadRightWasPressed()){
            turret.TURRET_FF_GAIN -= .01;
        }


//        turret.turretAngle = drive.imu.getHeading(AngleUnit.DEGREES);

        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent());
        shooter.update(Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY),shooter.currentFlywheelSpeed);
        intake.update();
        follower.update();
        turret.FFturret(follower.getHeading());

//        if(vision.llResult != null && vision.llResult.isValid()) {
////            Pose3D botPoseMt2 = llResult.getBotpose_MT2();
//            telemetry.addData("tx", vision.llResult.getTx());
//            telemetry.addData("ty", vision.llResult.getTy());
//            telemetry.addData("ta", vision.llResult.getTa());
//            shoot.speedCalc(vision.llResult.getTy());
//

//        }

        telemetry.addData("robotX velo", follower.getVelocity().getXComponent());
        telemetry.addData("robotY velo", follower.getVelocity().getYComponent());
        telemetry.addData("robot turn speed", turret.AngularVelocity(follower.getHeading()));
        telemetry.addData("FF", turret.turretFeedForwardServo);
        telemetry.addData("FF tuning", turret.TURRET_FF_GAIN);



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
        telemetry.addData("distance", Shooter.distance2D(follower.getPose().getX(), follower.getPose().getY(), turret.redGoalX,turret.redGoalY));
        telemetry.addData("blue Goal X", turret.blueGoalX);
        telemetry.addData("blue goal Y", turret.blueGoalY);

        telemetry.addData("analog", turret.analogangle);
        telemetry.addData("pos", turret.turretpos);

        telemetry.addData("index speed", intake.intakeR.getVelocity());






        telemetry.update();
        drive.driveFieldRelative(y,x,turn);
//        drive.driveRobotRelative(y,x,turn);
    }
}
